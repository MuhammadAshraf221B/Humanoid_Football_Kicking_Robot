"""
Ball Tracker → ESP32-CAM (UDP Servo) + Robot Arduino (Serial)
=============================================================
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
ESP32_CAM_IP  = "10.229.235.220"
STREAM_URL    = f"http://{ESP32_CAM_IP}:81/stream"
CONFIG_URL    = f"http://{ESP32_CAM_IP}/control"

ARDUINO_IP    = "10.229.235.220"
ARDUINO_PORT  = 4210

ROBOT_PORT    = "COM3"          # ← غيّرها للـ port الصح (Linux: /dev/ttyUSB0)
ROBOT_BAUD    = 115200

MODEL_PATH    = "yolov8n.pt"
#BALL_CLASS    = [32, 34, 35, 36, 37, 38, 39]
phone_CLASS    = [39]
CONF_THRESH   = 0.40
SEND_INTERVAL = 0.1             # ثانية بين كل UDP للسيرفو

SERVO_CENTER  = 90
SERVO_OFFSET  = 50

W_COOLDOWN    = 7.0             # ثواني استنى بعد أمر w
I_COOLDOWN    = 3.0             # ثواني استنى بعد أمر i

CAMERA_SETTINGS = {
    "framesize": 8, "quality": 6,
    "brightness": 1, "contrast": 1, "sharpness": 2,
    "awb": 1, "aec": 1, "agc": 1,
}
# ──────────────────────────────────────────────────────────────


def configure_camera():
    print("[INFO] Configuring ESP32-CAM...")
    for var, val in CAMERA_SETTINGS.items():
        try:
            requests.get(CONFIG_URL, params={"var": var, "val": val}, timeout=2)
        except Exception:
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


def send_robot(ser, cmd):
    try:
        ser.write(f"{cmd}\n".encode())
        print(f"[Robot] → '{cmd}'")
    except Exception as e:
        print(f"[Robot] Error: {e}")


# ─── SETUP ────────────────────────────────────────────────────
configure_camera()
yolo = YOLO(MODEL_PATH)
midas_model, midas_transform = load_midas()

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    robot_ser = serial.Serial(ROBOT_PORT, ROBOT_BAUD, timeout=1)
    time.sleep(2)
    print(f"[INFO] ✓ Robot connected on {ROBOT_PORT}")
except Exception as e:
    print(f"[WARN] Robot not available: {e}")
    robot_ser = None

cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)
if not cap.isOpened():
    raise SystemExit("[ERROR] Cannot open stream.")
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

print("[INFO] ✓ Streaming!  ESC=quit\n")

# ─── State ────────────────────────────────────────────────────
last_udp_send  = 0
last_robot_cmd = 0
waiting_for_i  = False      # True = بعتنا w وجاي i
frame_skip     = 0
depth_map      = None

# ─── MAIN LOOP ────────────────────────────────────────────────
while True:
    # أحدث فريم دايماً
    cap.grab()
    cap.grab()
    ret, frame = cap.retrieve()
    if not ret or frame is None:
        continue

    h, w = frame.shape[:2]
    cx_frame = w // 2

    # Depth كل 3 فريمات
    frame_skip += 1
    if frame_skip % 3 == 0:
        depth_map = get_depth_map(frame, midas_model, midas_transform)

    canvas = frame.copy()
    cv2.line(canvas, (cx_frame, 0), (cx_frame, h), (255, 255, 255), 1)

    # YOLO
    results  = yolo.predict(frame, conf=CONF_THRESH,
                             classes=phone_CLASS, verbose=False)[0]
    best_box = None
    best_conf = 0
    for box in results.boxes:
        c = float(box.conf[0])
        if c > best_conf:
            best_conf = c
            best_box  = box

    now = time.time()

    # ══════════════════════════════════════════════════════════
    if best_box is not None:

        # ── السيرفو (UDP) ──────────────────────────────────
        x1, y1, x2, y2 = map(int, best_box.xyxy[0].tolist())
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        error = cx - cx_frame

        if error > 20:
            target_angle = SERVO_CENTER + SERVO_OFFSET
            direction    = "RIGHT ►"
        elif error < -20:
            target_angle = SERVO_CENTER - SERVO_OFFSET
            direction    = "◄ LEFT"
        else:
            target_angle = SERVO_CENTER
            direction    = "● CENTER"

        if now - last_udp_send >= SEND_INTERVAL:
            udp.sendto(f"{target_angle}\n".encode(), (ARDUINO_IP, ARDUINO_PORT))
            last_udp_send = now
            print(f"[Ball] cx={cx}  err={error:+d}  "
                  f"angle={target_angle}°  {direction}")

        # ── الروبوت (Serial) ───────────────────────────────
        if robot_ser:
            elapsed = now - last_robot_cmd

            if not waiting_for_i:
                # ابعت w وابدأ العد
                send_robot(robot_ser, "w")
                last_robot_cmd = now
                waiting_for_i  = True

            elif elapsed >= W_COOLDOWN:
                # انتهت الـ 7 ثواني → ابعت i
                send_robot(robot_ser, "i")
                last_robot_cmd = now
                waiting_for_i  = False
                # بعد I_COOLDOWN هيرجع يبعت w تاني لأن waiting_for_i = False

            # ── label على الشاشة ──
            remaining = ""
            if waiting_for_i:
                remaining = f"i in {max(0, W_COOLDOWN - elapsed):.1f}s"
            else:
                remaining = f"w in {max(0, I_COOLDOWN - elapsed):.1f}s"

            cv2.putText(canvas, f"Robot: {remaining}",
                        (12, h - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 100), 2, cv2.LINE_AA)

        # ── رسم ───────────────────────────────────────────
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.circle(canvas, (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(canvas, (cx_frame, cy), (cx, cy), (0, 165, 255), 2)
        cv2.putText(canvas, f"Ball {best_conf:.2f}",
                    (x1, max(y1 - 22, 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas,
                    f"err={error:+d}px  angle={target_angle}°  {direction}",
                    (x1, max(y1 - 6, 34)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255), 1, cv2.LINE_AA)

    # ══════════════════════════════════════════════════════════
    else:
        # مفيش كورة → سيرفو على 90 وريست الـ robot timer
        if now - last_udp_send >= SEND_INTERVAL:
            udp.sendto(f"{SERVO_CENTER}\n".encode(), (ARDUINO_IP, ARDUINO_PORT))
            last_udp_send = now
            print("[No ball] → servo 90°")

        waiting_for_i  = False
        last_robot_cmd = 0

        cv2.putText(canvas, "No ball", (12, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 0, 255), 2, cv2.LINE_AA)

    cv2.putText(canvas, "ESC=quit",
                (12, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (180, 180, 180), 1)

    cv2.imshow("Ball Tracker", canvas)
    if cv2.waitKey(1) & 0xFF == 27:
        break

# ─── CLEANUP ──────────────────────────────────────────────────
cap.release()
udp.close()
if robot_ser:
    robot_ser.close()
cv2.destroyAllWindows()
print("[INFO] Done.")