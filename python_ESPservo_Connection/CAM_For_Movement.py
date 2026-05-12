"""
Ball Tracker → ESP32-CAM + Robot Serial Control
================================================
Dependencies:
pip install ultralytics opencv-python torch torchvision requests pyserial
"""

import cv2
import torch
import requests
import socket
import serial
import time
from ultralytics import YOLO

# ─── CONFIG ───────────────────────────────────────────────────
ESP32_CAM_IP  = "10.185.117.220"
STREAM_URL    = f"http://{ESP32_CAM_IP}:81/stream"
CONFIG_URL    = f"http://{ESP32_CAM_IP}/control"

ROBOT_PORT    = "COM7"
ROBOT_BAUD    = 115200

MODEL_PATH    = "yolov8n.pt"

BALL_CLASS    = [32, 34, 35, 36, 37, 38, 39]
CONF_THRESH   = 0.40

W_COOLDOWN    = 7.0
I_COOLDOWN    = 3.0

CAMERA_SETTINGS = {
    "framesize": 8,
    "quality": 6,
    "brightness": 1,
    "contrast": 1,
    "sharpness": 2,
    "awb": 1,
    "aec": 1,
    "agc": 1,
}

# ──────────────────────────────────────────────────────────────

def configure_camera():
    print("[INFO] Configuring ESP32-CAM...")
    for var, val in CAMERA_SETTINGS.items():
        try:
            requests.get(
                CONFIG_URL,
                params={"var": var, "val": val},
                timeout=2
            )
        except Exception:
            pass
    print("[INFO] ✓ Camera configured")


def load_midas():
    print("[INFO] Loading MiDaS...")
    model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
    model.eval()
    transform = torch.hub.load("intel-isl/MiDaS", "transforms").small_transform
    print("[INFO] ✓ MiDaS ready")
    return model, transform


def get_depth_map(frame, model, transform):
    inp = transform(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    with torch.no_grad():
        depth = model(inp)
        depth = torch.nn.functional.interpolate(
            depth.unsqueeze(1),
            size=frame.shape[:2],
            mode="bilinear",
            align_corners=False
        ).squeeze()
    return depth.cpu().numpy()


def send_robot(ser, cmd):
    try:
        ser.write(f"{cmd}\n".encode())
        print(f"[Robot] → {cmd}")
    except Exception as e:
        print(f"[Robot ERROR] {e}")


# ─── SETUP ────────────────────────────────────────────────────

configure_camera()

print("[INFO] Loading YOLO...")
yolo = YOLO(MODEL_PATH)
print("[INFO] ✓ YOLO ready")

midas_model, midas_transform = load_midas()

# ─── Serial Connection ────────────────────────────────────────

try:
    robot_ser = serial.Serial(ROBOT_PORT, ROBOT_BAUD, timeout=1)
    time.sleep(2)
    print(f"[INFO] ✓ Robot connected on {ROBOT_PORT}")
except Exception as e:
    print(f"[ERROR] Serial failed: {e}")
    robot_ser = None

# ─── Camera Stream ────────────────────────────────────────────

cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)

if not cap.isOpened():
    raise SystemExit("[ERROR] Cannot open stream")

cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
print("[INFO] ✓ Streaming started")

# ─── State ────────────────────────────────────────────────────

last_robot_cmd = 0
waiting_for_i  = False
frame_skip     = 0
depth_map      = None

# ─── MAIN LOOP ────────────────────────────────────────────────

while True:

    cap.grab()
    cap.grab()

    ret, frame = cap.retrieve()

    if not ret or frame is None:
        continue

    h, w = frame.shape[:2]
    frame_center = w // 2

    # ── Depth كل 3 فريمات ─────────────────────────────────
    frame_skip += 1
    if frame_skip % 3 == 0:
        depth_map = get_depth_map(frame, midas_model, midas_transform)

    canvas = frame.copy()

    # ── YOLO Detection ────────────────────────────────────
    results = yolo.predict(
        frame,
        conf=CONF_THRESH,
        classes=BALL_CLASS,
        verbose=False
    )[0]

    best_box  = None
    best_conf = 0

    for box in results.boxes:
        conf = float(box.conf[0])
        if conf > best_conf:
            best_conf = conf
            best_box  = box

    now = time.time()

    # ══════════════════════════════════════════════════════
    # Ball Found
    # ══════════════════════════════════════════════════════

    if best_box is not None:

        x1, y1, x2, y2 = map(int, best_box.xyxy[0].tolist())
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        error = cx - frame_center

        # ── Robot Control ───────────────────────────────
        if robot_ser:

            elapsed = now - last_robot_cmd

            # ✅ لو مش waiting_for_i، استنى I_COOLDOWN وابعت w
            if not waiting_for_i and elapsed >= I_COOLDOWN:
                send_robot(robot_ser, "w")
                last_robot_cmd = now
                waiting_for_i  = True

            # ✅ لو waiting_for_i، استنى W_COOLDOWN وابعت i
            # وبعدها waiting_for_i هترجع False فاللوب الجاي هيبعت w تاني
            elif waiting_for_i and elapsed >= W_COOLDOWN:
                send_robot(robot_ser, "i")
                last_robot_cmd = now
                waiting_for_i  = False

        # ── Draw ────────────────────────────────────────
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.circle(canvas, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(canvas, f"Ball {best_conf:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(canvas, "BALL DETECTED", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # ══════════════════════════════════════════════════════
    # No Ball — بلاش reset عشان الـ state يفضل زي ما هو
    # ══════════════════════════════════════════════════════

    else:
        cv2.putText(canvas, "NO BALL", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # ── UI ──────────────────────────────────────────────
    cv2.putText(canvas, "ESC = Quit", (20, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow("Ball Tracker", canvas)

    if cv2.waitKey(1) & 0xFF == 27:
        break

# ─── CLEANUP ──────────────────────────────────────────────────

cap.release()

if robot_ser:
    robot_ser.close()

cv2.destroyAllWindows()
print("[INFO] Done.")