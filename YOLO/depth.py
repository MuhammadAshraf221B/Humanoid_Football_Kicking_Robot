import cv2
import torch
import numpy as np
from ultralytics import YOLO

yolo = YOLO("yolov8n.pt")
midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
midas.eval()

transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
transform = transforms.small_transform

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = yolo(frame, verbose=False, classes=[32])
    boxes = results[0].boxes

    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    input_batch = transform(img_rgb)
    with torch.no_grad():
        depth = midas(input_batch)
        depth = torch.nn.functional.interpolate(
            depth.unsqueeze(1),
            size=frame.shape[:2],
            mode="bilinear",
            align_corners=False,
        ).squeeze()

    depth_map = depth.numpy()
    depth_norm = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
    depth_color = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_INFERNO)

    blended = cv2.addWeighted(frame, 0.6, depth_color, 0.4, 0)

    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx, cy = (x1+x2)//2, (y1+y2)//2
        ball_depth = depth_map[cy, cx]
        cv2.rectangle(blended, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.putText(blended, f"Depth: {ball_depth:.1f}", (x1, y1-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    cv2.imshow("Ball Depth Detection", blended)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()