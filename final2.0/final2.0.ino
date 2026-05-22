#include "Arduino.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define CAMERA_MODEL_WROVER_KIT
#include "camera_pins.h"

// ─────────────────────────────────────────────
//  WiFi
// ─────────────────────────────────────────────
const char *ssid     = "realme 8";
const char *password = "88888888";

// ─────────────────────────────────────────────
//  Web Status Server (port 8080)
// ─────────────────────────────────────────────
WebServer statusServer(8080);

// ─────────────────────────────────────────────
//  State Machine
// ─────────────────────────────────────────────
enum RobotState { IDLE, WALKING, KICKING, SEARCHING };
RobotState robotState = IDLE;
bool ballDetected     = false;

const char* stateToString(RobotState s) {
  switch (s) {
    case WALKING:   return "WALKING";
    case KICKING:   return "KICKING";
    case SEARCHING: return "SEARCHING";
    default:        return "IDLE";
  }
}

// ─────────────────────────────────────────────
//  PCA9685
// ─────────────────────────────────────────────
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define CH_LL2 0
#define CH_LL3 1
#define CH_LL5 2
#define CH_RL2 3
#define CH_RL3 4
#define CH_RL5 5
#define CH_LH1 6
#define CH_RH1 7

#define SERVOMIN 150
#define SERVOMAX 600

// ─────────────────────────────────────────────
//  Ultrasonic
// ─────────────────────────────────────────────
#define TRIG_PIN             12
#define ECHO_PIN              2
#define OBSTACLE_DISTANCE_CM 20

// ─────────────────────────────────────────────
//  Servo state
// ─────────────────────────────────────────────
int cur_LL2 = 105, cur_LL3 = 155, cur_LL5 = 85;
int cur_RL2 = 82,  cur_RL3 = 25,  cur_RL5 = 85;
int cur_LH1 = 120, cur_RH1 = 0;

// ─────────────────────────────────────────────
//  Forward declarations
// ─────────────────────────────────────────────
void startCameraServer();
void setupLedFlash();

// ══════════════════════════════════════════════
//  Servo helpers
// ══════════════════════════════════════════════
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setServo(int channel, int angle) {
  pca.setPWM(channel, 0, angleToPulse(angle));
}

void moveTo(int channel, int &cur, int target, int spd = 8) {
  if (cur < target)
    for (int i = cur; i <= target; i++) { setServo(channel, i); delay(spd); }
  else
    for (int i = cur; i >= target; i--) { setServo(channel, i); delay(spd); }
  cur = target;
}

// ══════════════════════════════════════════════
//  Ultrasonic
// ══════════════════════════════════════════════
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999.0;
  return duration * 0.034 / 2.0;
}

// ══════════════════════════════════════════════
//  Robot motions
// ══════════════════════════════════════════════
void initial_position() {
  Serial.println(">> Initial Position");
  robotState = IDLE;
  cur_LL2 = 105; cur_LL3 = 155; cur_LL5 = 85;
  cur_RL2 = 82;  cur_RL3 = 25;  cur_RL5 = 85;
  cur_LH1 = 120; cur_RH1 = 0;
  setServo(CH_LL2, 105);
  setServo(CH_LL3, 155);
  setServo(CH_LL5, 90);
  setServo(CH_RL2, 82);
  setServo(CH_RL3, 25);
  setServo(CH_RL5, 85);
  setServo(CH_LH1, 120);
  setServo(CH_RH1, 0);
}

void kick_right() {
  Serial.println(">> Kick Right");
  robotState = KICKING;
  moveTo(CH_LL5, cur_LL5, 78, 6);
  moveTo(CH_RL5, cur_RL5, 75, 6);
  delay(1000);
  moveTo(CH_LH1, cur_LH1, 100, 4);
  moveTo(CH_RL3, cur_RL3, 50, 4);
  delay(300);
  moveTo(CH_RL3, cur_RL3, 20, 5);
  delay(200);
  initial_position(); 
}

void stand_straight() {
  Serial.println(">> Stand");
  moveTo(CH_LL2, cur_LL2, 105);
  moveTo(CH_LL3, cur_LL3, 155);
  moveTo(CH_LL5, cur_LL5, 85);
  moveTo(CH_RL2, cur_RL2, 82);
  moveTo(CH_RL3, cur_RL3, 25);
  moveTo(CH_RL5, cur_RL5, 85);
}

void move_backward() {
  Serial.println(">> Move Backward");
  robotState = WALKING;
  moveTo(CH_LL5, cur_LL5, 95, 6);
  moveTo(CH_RL5, cur_RL5, 95, 6);
  delay(500);
  moveTo(CH_LL3, cur_LL3, 130, 6);
  delay(400);
  moveTo(CH_LL2, cur_LL2, 85, 6);
  delay(400);
  moveTo(CH_LL3, cur_LL3, 155, 6);
  delay(400);
  moveTo(CH_LL5, cur_LL5, 85, 6);
  moveTo(CH_RL5, cur_RL5, 85, 6);
  delay(500);
  moveTo(CH_LL5, cur_LL5, 75, 6);
  moveTo(CH_RL5, cur_RL5, 75, 6);
  delay(500);
  moveTo(CH_RL3, cur_RL3, 45, 6);
  delay(400);
  moveTo(CH_RL2, cur_RL2, 102, 6);
  delay(400);
  moveTo(CH_RL3, cur_RL3, 25, 6);
  delay(400);
  moveTo(CH_LL5, cur_LL5, 85, 6);
  moveTo(CH_RL5, cur_RL5, 85, 6);
  delay(500);
  initial_position();
}

void stand_up() {
  Serial.println(">> Stand Up");
  stand_straight();
}

void move_forward() {
  Serial.println(">> Move Forward");
  robotState = WALKING;
  moveTo(CH_LL5, cur_LL5, 95, 6);
  moveTo(CH_RL5, cur_RL5, 95, 6);
  delay(300);
  moveTo(CH_LL3, cur_LL3, 130, 6);
  delay(200);
  moveTo(CH_LL2, cur_LL2, 120, 6);
  delay(200);
  moveTo(CH_LL5, cur_LL5, 85, 6);
  moveTo(CH_RL5, cur_RL5, 85, 6);
  delay(300);
  moveTo(CH_LL5, cur_LL5, 75, 6);
  moveTo(CH_RL5, cur_RL5, 75, 6);
  delay(300);
  moveTo(CH_RL3, cur_RL3, 30, 6);
  delay(200);
  moveTo(CH_RL2, cur_RL2, 62, 6);
  delay(200);
  moveTo(CH_RL3, cur_RL3, 25, 6);
  delay(200);
  moveTo(CH_LL5, cur_LL5, 85, 6);
  moveTo(CH_RL5, cur_RL5, 85, 6);
  delay(300);
  initial_position();
}

void turn_right() {
  Serial.println(">> Turn Right");
  robotState = WALKING;
  moveTo(CH_LL5, cur_LL5, 95, 6);
  delay(50);
  moveTo(CH_RL3, cur_RL3, 10, 4);
  delay(100);
  moveTo(CH_RL3, cur_RL3, 30, 4);
  delay(100);
  initial_position();
}

void turn_left() {
  Serial.println(">> Turn Left");
  robotState = WALKING;
  moveTo(CH_RL5, cur_RL5, 95, 6);
  delay(50);
  moveTo(CH_LL3, cur_LL3, 125, 5);
  delay(100);
  moveTo(CH_LL3, cur_LL3, 150, 5);
  initial_position();
}

void move_forward_with_hand() {
  Serial.println(">> Move Forward With Hand");
  robotState = WALKING;
  moveTo(CH_LL5, cur_LL5, 95, 6);
  moveTo(CH_RL5, cur_RL5, 95, 6);
  delay(300);
  moveTo(CH_LL3, cur_LL3, 130, 6);
  moveTo(CH_RH1, cur_RH1, 50, 6);
  delay(200);
  moveTo(CH_LL2, cur_LL2, 130, 6);
  delay(200);
  moveTo(CH_LL5, cur_LL5, 85, 6);
  moveTo(CH_RL5, cur_RL5, 85, 6);
  delay(300);
  moveTo(CH_LL5, cur_LL5, 80, 6);
  moveTo(CH_RL5, cur_RL5, 80, 6);
  delay(300);
  moveTo(CH_RL3, cur_RL3, 30, 6);
  moveTo(CH_LH1, cur_LH1, 80, 6);
  delay(200);
  moveTo(CH_RL2, cur_RL2, 52, 6);
  delay(200);
  moveTo(CH_RL3, cur_RL3, 25, 6);
  delay(200);
  moveTo(CH_LL5, cur_LL5, 85, 6);
  moveTo(CH_RL5, cur_RL5, 85, 6);
  delay(300);
  initial_position();
}

void move_hands() {
  Serial.println(">> Move Hands");
  moveTo(CH_RH1, cur_RH1, 80, 1);
  // delay(6000);
  // moveTo(CH_LH1, cur_LH1, 70, 6);
  // delay(6000);
  // initial_position();
}

// ══════════════════════════════════════════════
//  HTTP /status handler
// ══════════════════════════════════════════════
void handleStatus() {
  float dist = getDistance();
  String json = "{";
  json += "\"state\":\"" + String(stateToString(robotState)) + "\",";
  json += "\"ball\":"    + String(ballDetected ? "true" : "false") + ",";
  json += "\"distance\":" + String(dist, 1);
  json += "}";
  statusServer.sendHeader("Access-Control-Allow-Origin", "*");
  statusServer.send(200, "application/json", json);
}

// ══════════════════════════════════════════════
//  Serial command parser
// ══════════════════════════════════════════════
void printCommands() {
  Serial.println("=== Commands ===");
  Serial.println("  i  -> Initial Position");
  Serial.println("  s  -> Kick Right");
  Serial.println("  b  -> Move Backward");
  Serial.println("  u  -> Stand Up");
  Serial.println("  w  -> Move Forward");
  Serial.println("  r  -> Turn Right");
  Serial.println("  l  -> Turn Left");
  Serial.println("  h  -> Move Forward With Hand");
  Serial.println("  p  -> Move Hands");
  Serial.println("  LL2/LL3/LL5/RL2/RL3/RL5 [angle]");
}

void handleSerialCommand(String cmd) {
  cmd.trim();

  if (cmd == "h" || cmd == "r" || cmd == "l" || cmd == "s") {
    float dist = getDistance();
    Serial.print("[ULTRASONIC] Distance: ");
    Serial.print(dist);
    Serial.println(" cm");
    if (dist > 0 && dist < OBSTACLE_DISTANCE_CM) {
      Serial.println("[OBSTACLE] Too close! Command blocked.");
      Serial.println("OBSTACLE");
      initial_position();
      return;
    }
  }

  if      (cmd == "i") initial_position();
  else if (cmd == "s") kick_right();
  else if (cmd == "b") move_backward();
  else if (cmd == "u") stand_up();
  else if (cmd == "w") move_forward();
  else if (cmd == "r") turn_right();
  else if (cmd == "l") turn_left();
  else if (cmd == "h") move_forward_with_hand();
  else if (cmd == "p") move_hands();
  else {
    int sp = cmd.indexOf(' ');
    if (sp != -1) {
      String name  = cmd.substring(0, sp);
      int    angle = constrain(cmd.substring(sp + 1).toInt(), 0, 180);
      if      (name == "LL2") moveTo(CH_LL2, cur_LL2, angle);
      else if (name == "LL3") moveTo(CH_LL3, cur_LL3, angle);
      else if (name == "LL5") moveTo(CH_LL5, cur_LL5, angle);
      else if (name == "RL2") moveTo(CH_RL2, cur_RL2, angle);
      else if (name == "RL3") moveTo(CH_RL3, cur_RL3, angle);
      else if (name == "RL5") moveTo(CH_RL5, cur_RL5, angle);
      else { Serial.println("Unknown!"); printCommands(); return; }
      Serial.print("Moved "); Serial.print(name);
      Serial.print(" -> "); Serial.println(angle);
    } else {
      Serial.println("Unknown command!");
      printCommands();
    }
  }
}

// ══════════════════════════════════════════════
//  setup
// ══════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // ── Camera ───────────────────────────────────
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count     = 2;
      config.grab_mode    = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size  = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  // ── WiFi ─────────────────────────────────────
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP: http://");
  Serial.println(WiFi.localIP());

  // ── Camera stream server ──────────────────────
  startCameraServer();
  Serial.print("Stream: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");

  // ── Status server ─────────────────────────────
  statusServer.on("/status", handleStatus);
  statusServer.begin();
  Serial.print("Dashboard: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":8080/status");

  // ── PCA9685 init (SDA=13, SCL=14) ───────────
  Wire.begin(13, 14);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(50);
  delay(100);

  initial_position();
  Serial.println("====== Robot Ready ======");
  printCommands();
}

// ══════════════════════════════════════════════
//  loop
// ══════════════════════════════════════════════
void loop() {
  statusServer.handleClient();   // <── handles WiFi dashboard requests

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleSerialCommand(cmd);
  }

  delay(10);
}
