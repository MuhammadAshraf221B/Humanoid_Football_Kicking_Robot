import cv2
import socket
import serial
import time
from ultralytics import YOLO

# ─── CONFIG ───────────────────────────────────────────────────
ESP32_IP      = "10.229.235.220" 
UDP_PORT      = 4210
ROBOT_PORT    = "COM5" 
ROBOT_BAUD    = 115200
MODEL_PATH    = "yolov8n.pt"
BALL_CLASS    = [32] 
CONF_THRESH   = 0.50

# توقيتات الدورة
WALK_DURATION = 4.0  # مدة المشي
IDLE_DURATION = 0.5  # مدة الثبات قبل الفحص التالي
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

# متغيرات الحالة
cycle_state = "SEARCHING" 
last_state_change = 0

print("[INFO] Intelligent Continuous Loop Active.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # الكشف عن الكورة
    results = model.predict(frame, conf=CONF_THRESH, classes=BALL_CLASS, verbose=False)[0]
    target_detected = len(results.boxes) > 0
    now = time.time()
    canvas = frame.copy()

    # --- منطق الدورة المستمرة ---

    # 1. حالة البحث (أو انتظار الكورة)
    if cycle_state == "SEARCHING":
        if target_detected:
            if robot_ser:
                robot_ser.write(b"w\n")
                print("[Action] → Ball Found! Sending 'w' (Walking 5s)")
            cycle_state = "WALKING"
            last_state_change = now

    # 2. حالة المشي (تجاهل التقطيع لمدة 5 ثوانٍ)
    elif cycle_state == "WALKING":
        if now - last_state_change >= WALK_DURATION:
            if robot_ser:
                robot_ser.write(b"i\n")
                print("[Action] → Time up. Sending 'i' (Resting 3s)")
            cycle_state = "RESTING"
            last_state_change = now

    # 3. حالة الثبات (انتظار 3 ثوانٍ ثم العودة للبحث فوراً)
    elif cycle_state == "RESTING":
        if now - last_state_change >= IDLE_DURATION:
            # هنا الميزة: بمجرد انتهاء الوقت، نعود لحالة البحث 
            # لو الكورة لسه موجودة، هيدخل في "WALKING" في اللفة اللي بعدها فوراً
            cycle_state = "SEARCHING"
            print("[System] → Ready to check for ball again...")

    # --- عرض البيانات على الشاشة ---
    status_color = (0, 255, 0) if target_detected else (0, 0, 255)
    cv2.putText(canvas, f"Mode: {cycle_state}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

    if target_detected:
        box = results.boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (255, 0, 0), 2)

    cv2.imshow("Continuous Tracker", canvas)
    
    if cv2.waitKey(1) & 0xFF == 27: break

cap.release()
cv2.destroyAllWindows()
if robot_ser: robot_ser.close()
udp_sock.close()