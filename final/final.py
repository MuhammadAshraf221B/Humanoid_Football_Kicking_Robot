import cv2
import socket
import serial
import time
from ultralytics import YOLO

# ─── CONFIG ───────────────────────────────────────────────────
ESP32_IP      = "10.229.235.220" 
UDP_PORT      = 4210
ROBOT_PORT    = "COM7" 
ROBOT_BAUD    = 115200
MODEL_PATH    = "yolov8n.pt"
BALL_CLASS    = [32] 
CONF_THRESH   = 0.50

# توقيتات الدورة المطلوبة
WALK_DURATION = 5.0  # مدة إرسال w والانتظار
IDLE_DURATION = 3.0  # مدة إرسال i والانتظار
# ──────────────────────────────────────────────────────────────

model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(0)
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    robot_ser = serial.Serial(ROBOT_PORT, ROBOT_BAUD, timeout=0.1)
    time.sleep(2)
    print(f"[INFO] ✓ Connected to Robot on {ROBOT_PORT}")
except Exception as e:
    print(f"[ERROR] Serial Connection Failed: {e}")
    robot_ser = None

# متغيرات الحالة "الذكية"
cycle_state = "IDLE"  # الحالات: IDLE (انتظار), WALKING (مشي), STOPPING (توقف)
last_state_change = 0

print("[INFO] Intelligent Cycle Ready. Searching for Ball...")

while True:
    ret, frame = cap.read()
    if not ret: break

    results = model.predict(frame, conf=CONF_THRESH, classes=BALL_CLASS, verbose=False)[0]
    target_detected = len(results.boxes) > 0
    now = time.time()
    canvas = frame.copy()

    # --- منطق الدورة الذكية (The Smart State Machine) ---
    
    # 1. حالة الانتظار (IDLE): إذا رأى الكورة، يبدأ المشي فوراً
    if cycle_state == "IDLE":
        if target_detected:
            if robot_ser:
                robot_ser.write(b"w\n")
                print("[Cycle] → Sending 'w' (Walking for 5s)")
            cycle_state = "WALKING"
            last_state_change = now

    # 2. حالة المشي (WALKING): ينتظر 5 ثوانٍ بغض النظر عن الـ Detection
    elif cycle_state == "WALKING":
        if now - last_state_change >= WALK_DURATION:
            if robot_ser:
                robot_ser.write(b"i\n")
                print("[Cycle] → Sending 'i' (Initial Pos for 3s)")
            cycle_state = "STOPPING"
            last_state_change = now

    # 3. حالة التوقف (STOPPING): ينتظر 3 ثوانٍ ثم يعود لطلب ديدكشن جديد
    elif cycle_state == "STOPPING":
        if now - last_state_change >= IDLE_DURATION:
            print("[Cycle] → Cycle Complete. Looking for ball again...")
            cycle_state = "IDLE"

    # --- الجزء المرئي وتتبع الرأس (UDP) يظل شغالاً دائماً ---
    if target_detected:
        box = results.boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        # إرسال الزوايا للرأس (اختياري، يجعله يتبع الكورة حتى وهو واقف)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(canvas, f"STATE: {cycle_state}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(canvas, f"SEARCHING... (State: {cycle_state})", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("Smart Cycle Tracker", canvas)
    if cv2.waitKey(1) & 0xFF == 27: break

cap.release()
cv2.destroyAllWindows()
if robot_ser: robot_ser.close()
udp_sock.close()