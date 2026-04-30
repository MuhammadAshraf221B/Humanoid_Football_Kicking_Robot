import socket
import struct
import cv2
import numpy as np
import time

HOST = "0.0.0.0"
PORT = 5000

def recv_all(sock, size):
    data = b""
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet:
            return None
        data += packet
    return data

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)

print("Waiting ESP32...")

conn, addr = server.accept()
print("Connected:", addr)

fps_time = time.time()
count = 0

while True:
    try:
        raw_len = recv_all(conn, 4)
        if not raw_len:
            break

        frame_len = struct.unpack("<I", raw_len)[0]

        frame_data = recv_all(conn, frame_len)
        if not frame_data:
            break

        frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)

        if frame is None:
            continue
const char* ssid = "WE_652DA4";
const char* password = "n7j02656";
        count += 1

        if time.time() - fps_time >= 1:
            fps = count
            count = 0
            fps_time = time.time()

        cv2.putText(frame, f"FPS: {fps}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("ESP32 Stream", frame)

        if cv2.waitKey(1) == 27:
            break

    except:
        break

conn.close()
server.close()
cv2.destroyAllWindows()