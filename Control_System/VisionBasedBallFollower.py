import cv2
import time
import serial
import numpy as np
from ultralytics import YOLO

MODEL_PATH = "yolov8s.pt"

ROBOT_PORT = "COM5"
ROBOT_BAUD = 115200

CONFIDENCE_THRESHOLD = 0.60
YOLO_INTERNAL_CONF = 0.45
YOLO_IMGSZ = 960

UPDATE_INTERVAL = 3.0
KICK_Y_THRESHOLD = 0.75
CENTER_TOLERANCE = 90
KICK_DURATION = 1.0

USE_FALLBACK = False

GUI_HOLD_SECONDS = 0.35
ROI_SCALE = 2.2

MIN_CONTOUR_AREA = 1500
MIN_CIRCULARITY = 0.80
MIN_FILL_RATIO = 0.75

OBSTACLE_PAUSE_DURATION = 2.0

try:
    robot_ser = serial.Serial(ROBOT_PORT, ROBOT_BAUD, timeout=0.1)
    time.sleep(2)
    print(f"[INFO] ✓ Connected to Robot on {ROBOT_PORT}")
except Exception as e:
    print(f"[ERROR] Serial Connection Failed: {e}")
    robot_ser = None

is_kicking = False
kick_start_time = 0

last_time = 0

last_seen = None
last_seen_time = 0.0

obstacle_detected = False
obstacle_time = 0.0

def send_command(cmd):
    global obstacle_detected, obstacle_time

    serial_map = {
        "WALK_FORWARD": b"h\n",
        "TURN_RIGHT":   b"r\n",
        "TURN_LEFT":    b"l\n",
        "KICK":         b"s\n",
        "SEARCHING":    b"f\n",
    }

    if obstacle_detected:
        if time.time() - obstacle_time < OBSTACLE_PAUSE_DURATION:
            print(f"[OBSTACLE] Waiting... Command '{cmd}' ignored.")
            return
        else:
            obstacle_detected = False

    print(f"[ROBOT] {cmd}")

    if robot_ser and cmd in serial_map:
        robot_ser.write(serial_map[cmd])

        time.sleep(0.05)

        if robot_ser.in_waiting:
            try:
                feedback = robot_ser.readline().decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if feedback == "OBSTACLE":
                    print("[OBSTACLE] Arduino reported obstacle!")
                    obstacle_detected = True
                    obstacle_time = time.time()

            except Exception as e:
                print(f"[WARN] Serial read error: {e}")

def contour_score(cnt):

    area = cv2.contourArea(cnt)

    if area <= 0:
        return None

    peri = cv2.arcLength(cnt, True)

    if peri <= 0:
        return None

    circularity = 4 * np.pi * (area / (peri * peri))

    x, y, w, h = cv2.boundingRect(cnt)

    bbox_area = w * h

    if bbox_area <= 0:
        return None

    if bbox_area > 50000:
        return None

    aspect_ratio = w / float(h)

    if aspect_ratio < 0.75 or aspect_ratio > 1.25:
        return None

    fill = area / bbox_area

    if area < MIN_CONTOUR_AREA:
        return None

    if circularity < MIN_CIRCULARITY:
        return None

    if fill < MIN_FILL_RATIO:
        return None

    score = area * (
        (0.7 + 0.3 * circularity) *
        (0.7 + 0.3 * fill)
    )

    return score, (x, y, w, h)

def fallback_bbox(frame_bgr, roi=None):

    if roi is None:
        x0, y0, x1, y1 = (
            0,
            0,
            frame_bgr.shape[1],
            frame_bgr.shape[0]
        )
    else:
        x0, y0, x1, y1 = roi

    sub = frame_bgr[y0:y1, x0:x1]

    if sub.size == 0:
        return None

    gray = cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gray, (11, 11), 0)

    _, thresh = cv2.threshold(
        blur,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(
        thresh,
        cv2.MORPH_OPEN,
        kernel,
        iterations=1
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=2
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    best = None
    best_score = -1

    for cnt in contours:

        out = contour_score(cnt)

        if out is None:
            continue

        score, (x, y, w, h) = out

        if score > best_score:
            best_score = score
            best = (x, y, w, h)

    if best is None:
        return None

    x, y, w, h = best

    fx1, fy1 = x0 + x, y0 + y
    fx2, fy2 = x0 + x + w, y0 + y + h

    fx1 = max(0, fx1)
    fy1 = max(0, fy1)

    fx2 = min(frame_bgr.shape[1] - 1, fx2)
    fy2 = min(frame_bgr.shape[0] - 1, fy2)

    return (fx1, fy1, fx2, fy2)

def get_roi_around_last(frame_w, frame_h, last_box):

    x1, y1, x2, y2 = last_box

    bw = x2 - x1
    bh = y2 - y1

    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2

    rw = int(max(60, bw * ROI_SCALE))
    rh = int(max(60, bh * ROI_SCALE))

    sx1 = max(0, cx - rw // 2)
    sy1 = max(0, cy - rh // 2)

    sx2 = min(frame_w, cx + rw // 2)
    sy2 = min(frame_h, cy + rh // 2)

    return (sx1, sy1, sx2, sy2)

model = YOLO(MODEL_PATH)

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("=== BALL DETECTION STARTED ===")

while cap.isOpened():

    ret, frame = cap.read()

    if not ret:
        break

    h, w, _ = frame.shape

    center_x = w // 2

    now = time.time()

    if robot_ser and robot_ser.in_waiting:

        try:
            line = robot_ser.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if line == "OBSTACLE":
                print("[OBSTACLE] Detected!")
                obstacle_detected = True
                obstacle_time = now

        except Exception:
            pass

    results = model.predict(
        frame,
        conf=YOLO_INTERNAL_CONF,
        classes=[32],
        imgsz=YOLO_IMGSZ,
        verbose=False
    )

    ball_found = False

    final_cx = None
    final_cy = None

    x1 = y1 = x2 = y2 = None

    if (
        results and
        results[0].boxes is not None and
        len(results[0].boxes) > 0
    ):

        boxes = results[0].boxes

        filtered = [
            b for b in boxes
            if float(b.conf[0]) >= CONFIDENCE_THRESHOLD
        ]

        if filtered:

            box = max(
                filtered,
                key=lambda b:
                (b.xyxy[0][2] - b.xyxy[0][0]) *
                (b.xyxy[0][3] - b.xyxy[0][1])
            )

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            final_cx = (x1 + x2) // 2
            final_cy = y2

            ball_found = True

    if (not ball_found) and USE_FALLBACK:

        fb = None

        if last_seen is not None:

            roi_search = get_roi_around_last(
                w,
                h,
                last_seen
            )

            fb = fallback_bbox(frame, roi_search)

        if fb is not None:

            x1, y1, x2, y2 = fb

            final_cx = (x1 + x2) // 2
            final_cy = y2

            ball_found = True

    if ball_found and x1 is not None:

        last_seen = (x1, y1, x2, y2)
        last_seen_time = now

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            "BALL",
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

    else:

        if (
            last_seen is not None and
            (now - last_seen_time) <= GUI_HOLD_SECONDS
        ):

            x1, y1, x2, y2 = last_seen

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

    if obstacle_detected and (
        now - obstacle_time < OBSTACLE_PAUSE_DURATION
    ):

        cv2.putText(
            frame,
            "!! OBSTACLE DETECTED !!",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

    if is_kicking:

        if now - kick_start_time > KICK_DURATION:
            is_kicking = False

        ball_found = False

    if (
        ball_found and
        not is_kicking and
        now - last_time > UPDATE_INTERVAL
    ):

        if (
            final_cy > h * KICK_Y_THRESHOLD and
            abs(final_cx - center_x) < CENTER_TOLERANCE
        ):

            send_command("KICK")

            is_kicking = True
            kick_start_time = now

        elif final_cx < center_x - CENTER_TOLERANCE:

            send_command("TURN_LEFT")

        elif final_cx > center_x + CENTER_TOLERANCE:

            send_command("TURN_RIGHT")

        else:

            send_command("WALK_FORWARD")

        last_time = now

    elif not ball_found and not is_kicking:

        send_command("SEARCHING")

    cv2.line(
        frame,
        (center_x - CENTER_TOLERANCE, 0),
        (center_x - CENTER_TOLERANCE, h),
        (255, 0, 0),
        1
    )

    cv2.line(
        frame,
        (center_x + CENTER_TOLERANCE, 0),
        (center_x + CENTER_TOLERANCE, h),
        (255, 0, 0),
        1
    )

    cv2.line(
        frame,
        (0, int(h * KICK_Y_THRESHOLD)),
        (w, int(h * KICK_Y_THRESHOLD)),
        (0, 0, 255),
        2
    )

    cv2.imshow("Ball Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()

cv2.destroyAllWindows()

if robot_ser:
    robot_ser.close()