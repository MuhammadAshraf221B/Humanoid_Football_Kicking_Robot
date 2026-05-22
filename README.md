# Humanoid Football Kicking Robot

## Overview
This project is an AI-powered humanoid robot that detects, tracks, and kicks a football using YOLOv8, ESP32-CAM, servo control, and real-time decision making.  
It combines computer vision, embedded systems, and control algorithms.

---

## System Architecture

Camera → YOLOv8 → Python Decision System → Serial Communication → ESP32 Robot → Servo Motors

---

## Features

- Real-time football detection using YOLOv8
- Autonomous walking and kicking behavior
- Obstacle detection using ultrasonic sensor
- Voice command control
- ESP32-CAM live streaming
- Web dashboard for monitoring robot state
- Finite State Machine (IDLE, WALKING, SEARCHING, KICKING)

---

## Hardware Components

- ESP32-CAM
- PCA9685 Servo Driver
- Servo Motors
- Ultrasonic Sensor (HC-SR04)
- Humanoid Robot Frame
- Power Supply

---

## Software Stack

- Python (OpenCV, YOLOv8, PySerial)
- Arduino IDE (ESP32 firmware)
- HTML / CSS / JavaScript (Dashboard)
- Simulink (Modeling & Simulation)

---

## Robot States

- IDLE: Waiting for commands
- WALKING: Moving toward ball
- SEARCHING: Looking for ball
- KICKING: Performing kick action

---

## Movement Commands

h → Move forward  
b → Move backward  
r → Turn right  
l → Turn left  
s → Kick  
i → Initial position  
p → Move hands  

---

## AI Detection

- Model: YOLOv8 (yolov8s.pt)
- Detects football in real-time
- Calculates ball position
- Sends movement commands to ESP32

---

## Voice Control

Supported commands:
- forward
- back
- left
- right
- shoot
- hand
- stand

---

## Obstacle Avoidance

- Ultrasonic sensor measures distance
- If distance < 20 cm:
  - Robot stops movement
  - Blocks unsafe commands

---

## Web Dashboard

Displays:
- Robot state
- Ball detection status
- Distance readings
- Connection status
- System logs

API:
http://<ESP32-IP>:8080/status

---

## ESP32-CAM Stream

http://<ESP32-IP>/stream

---

## Installation

pip install ultralytics opencv-python pyserial numpy SpeechRecognition pyaudio

---

## Running Project

1. Upload ESP32 code using Arduino IDE
2. Run Python ball detection script
3. Run voice control script
4. Open dashboard in browser

---

## Simulink

Used for:
- Motion modeling
- Control system design
- System simulation

---

## Future Improvements

- Reinforcement learning control
- ROS integration
- SLAM navigation
- Autonomous goalkeeper mode
- Mobile app control

---

## Author

Mohamed Ashraf  
AI & Machine Learning Student

---

## License

For educational purposes only
