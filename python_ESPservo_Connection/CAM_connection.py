"""
Ball Tracker → Arduino via UDP (WiFi)
======================================
ESP32-CAM → YOLO → X position → UDP → Arduino/ESP32 → Servo Pan

Dependencies:
    pip install ultralytics opencv-python torch torchvision requests
"""

import cv2
import torch
import numpy as np
import requests
import socket
import time
from ultralytics import YOLO

# ─── CONFIG ───────────────────────────────────────────────────────────────────
ESP32_CAM_IP   = "192.168.1.11"
STREAM_URL     = f"http://{ESP32_CAM_IP}:81/stream"
CONFIG_URL     = f"http://{ESP32_CAM_IP}/control"

ARDUINO_IP     = "192.168.1.11"
ARDUINO_PORT   = 4210

MODEL_PATH     = "yolov8n.pt"
BALL_CLASS     = [32, 34, 35, 36, 37, 38, 39]
CONF_THRESH    = 0.40
SEND_INTERVAL  = 0.1

SERVO_CENTER   = 90
SERVO_OFFSET   = 50

CAMERA_SETTINGS = {
    "framesize": 8, "quality": 6,
    "brightness": 1, "contrast": 1, "sharpness": 2,
    "awb": 1, "aec": 1, "agc": 1,
}
# ──────────────────────────────────────────────────────────────────────────────

def configure_camera():
    print("[INFO] Configuring ESP32-CAM...")
    for var, val in CAMERA_SETTINGS.items():
        try:
            requests.get(CONFIG_URL, params={"var": var, "val": val}, timeout=2)
        except:
            pass
    print("[INFO] ✓ Camera configured")

def load_midas():
    print("[INFO] Loading MiDaS...")
    m = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
    m.eval()
    t = torch.hub.load("intel-isl/MiDaS", "transforms").small_transform
    print("[INFO] ✓ MiDaS ready")
    return m, t

def get_depth_map(frame, midas_model, transform):
    inp = transform(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    with torch.no_grad():
        d = midas_model(inp)
        d = torch.nn.functional.interpolate(
            d.unsqueeze(1), size=frame.shape[:2],
            mode="bilinear", align_corners=False
        ).squeeze()
    return d.cpu().numpy()

# ─── SETUP ────────────────────────────────────────────────────────────────────
configure_camera()

yolo = YOLO(MODEL_PATH)
midas_model, midas_transform = load_midas()

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

cap = cv2.VideoCapture(STREAM_URL)
if not cap.isOpened():
    raise SystemExit("[ERROR] Cannot open stream.")

# ── تحسين الفريمات ──
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # buffer صغير = أحدث فريم دايماً

print("[INFO] ✓ Streaming!  ESC=quit\n")

last_send  = 0
frame_skip = 0  # عداد لـ skip الـ depth

# ─── MAIN LOOP ────────────────────────────────────────────────────────────────
while True:
    # فضّي الـ buffer واخد أحدث فريم
    cap.grab()
    cap.grab()
    ret, frame = cap.retrieve()
    if not ret or frame is None:
        continue

    h, w = frame.shape[:2]
    frame_center_x = w // 2

    # ── Depth بس على كل 3 فريمات عشان يخف الـ load ──
    frame_skip += 1
    if frame_skip % 3 == 0:
        depth_map = get_depth_map(frame, midas_model, midas_transform)

    # ── Canvas = الفريم الأصلي بدون أي overlay ──
    canvas = frame.copy()

    # خط المركز
    cv2.line(canvas, (frame_center_x, 0), (frame_center_x, h), (255, 255, 255), 1)

    # YOLO
    results   = yolo.predict(frame, conf=CONF_THRESH, classes=BALL_CLASS, verbose=False)[0]
    detected  = False
    best_box  = None
    best_conf = 0

    for box in results.boxes:
        c = float(box.conf[0])
        if c > best_conf:
            best_conf = c
            best_box  = box

    now = time.time()

    if best_box is not None:
        x1, y1, x2, y2 = map(int, best_box.xyxy[0].tolist())
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        error  = cx - frame_center_x

        if error > 20:
            target_angle = SERVO_CENTER + SERVO_OFFSET
            direction    = "RIGHT ►"
        elif error < -20:
            target_angle = SERVO_CENTER - SERVO_OFFSET
            direction    = "◄ LEFT"
        else:
            target_angle = SERVO_CENTER
            direction    = "● CENTER"

        if now - last_send >= SEND_INTERVAL:
            msg = f"{target_angle}\n".encode()
            udp.sendto(msg, (ARDUINO_IP, ARDUINO_PORT))
            last_send = now
            print(f"[Ball] cx={cx}  err={error:+d}  angle={target_angle}°  {direction}  → UDP sent")

        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.circle(canvas,    (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(canvas, (frame_center_x, cy), (cx, cy), (0, 165, 255), 2)

        cv2.putText(canvas, f"Ball {best_conf:.2f}", (x1, max(y1-22, 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, f"err={error:+d}px  angle={target_angle}°  {direction}",
                    (x1, max(y1-6, 34)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        detected = True

    if not detected:
        if now - last_send >= SEND_INTERVAL:
            msg = f"{SERVO_CENTER}\n".encode()
            udp.sendto(msg, (ARDUINO_IP, ARDUINO_PORT))
            last_send = now
            print("[No ball] → servo reset to 90°")

        cv2.putText(canvas, "No ball", (12, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.putText(canvas, "ESC=quit",
                (12, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    cv2.imshow("Ball Tracker (UDP → Arduino)", canvas)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
udp.close()
cv2.destroyAllWindows()
print("[INFO] Done.")