#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "esp_http_server.h"

// ─── WiFi ─────────────────────────────────────────────────────
const char* ssid     = "Infinix HOT 11S";
const char* password = "123456789";

// ─── PCA9685 ──────────────────────────────────────────────────
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define CH_LL2 0
#define CH_LL3 1
#define CH_LL5 2
#define CH_RL2 3
#define CH_RL3 4
#define CH_RL5 5

#define SERVOMIN 150
#define SERVOMAX 600

// ─── Camera Pins ──────────────────────────────────────────────
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       32
#define PCLK_GPIO_NUM     33
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23

// ─── Servo State ──────────────────────────────────────────────
int cur_LL2 = 105, cur_LL3 = 155, cur_LL5 = 85;
int cur_RL2 = 82,  cur_RL3 = 25,  cur_RL5 = 85;

// ─── Servo Functions ──────────────────────────────────────────
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setServo(int channel, int angle) {
  pca.setPWM(channel, 0, angleToPulse(angle));
}

void moveTo(int channel, int &cur, int target, int spd = 8) {
  if (cur < target) {
    for (int i = cur; i <= target; i++) { setServo(channel, i); delay(spd); }
  } else {
    for (int i = cur; i >= target; i--) { setServo(channel, i); delay(spd); }
  }
  cur = target;
}

// ─── Robot Movements ──────────────────────────────────────────
void initial_position() {
  Serial.println(">> Initial Position");
  cur_LL2 = 105; cur_LL3 = 155; cur_LL5 = 85;
  cur_RL2 = 82;  cur_RL3 = 25;  cur_RL5 = 85;
  setServo(CH_LL2, 105); setServo(CH_LL3, 155); setServo(CH_LL5, 85);
  setServo(CH_RL2, 82);  setServo(CH_RL3, 25);  setServo(CH_RL5, 85);
}

void stand_straight() {
  Serial.println(">> Stand");
  moveTo(CH_LL2, cur_LL2, 105); moveTo(CH_LL3, cur_LL3, 155); moveTo(CH_LL5, cur_LL5, 85);
  moveTo(CH_RL2, cur_RL2, 82);  moveTo(CH_RL3, cur_RL3, 25);  moveTo(CH_RL5, cur_RL5, 85);
}

void sit_down() {
  Serial.println(">> Sit Down");
  moveTo(CH_LL3, cur_LL3, 180);
  moveTo(CH_RL3, cur_RL3, 50);
}

void stand_up() {
  Serial.println(">> Stand Up");
  stand_straight();
}

void move_forward() {
  Serial.println(">> Move Forward");
  moveTo(CH_LL5, cur_LL5, 95, 6); moveTo(CH_RL5, cur_RL5, 95, 6); delay(500);
  moveTo(CH_LL3, cur_LL3, 130, 6); delay(400);
  moveTo(CH_LL2, cur_LL2, 120, 6); delay(400);
  moveTo(CH_LL5, cur_LL5, 85, 6); moveTo(CH_RL5, cur_RL5, 85, 6); delay(500);
  moveTo(CH_LL5, cur_LL5, 75, 6); moveTo(CH_RL5, cur_RL5, 75, 6); delay(500);
  moveTo(CH_RL3, cur_RL3, 30, 6); delay(400);
  moveTo(CH_RL2, cur_RL2, 62, 6); delay(400);
  moveTo(CH_RL3, cur_RL3, 25, 6); delay(400);
  moveTo(CH_LL5, cur_LL5, 85, 6); moveTo(CH_RL5, cur_RL5, 85, 6); delay(500);
}

void turn_right() {
  Serial.println(">> Turn Right");
  moveTo(CH_RL5, cur_RL5, 60); moveTo(CH_LL5, cur_LL5, 60);
  delay(300);
  moveTo(CH_RL5, cur_RL5, 85); moveTo(CH_LL5, cur_LL5, 85);
}

void turn_left() {
  Serial.println(">> Turn Left");
  moveTo(CH_LL5, cur_LL5, 95, 6); moveTo(CH_RL5, cur_RL5, 95, 6); delay(500);
  moveTo(CH_LL3, cur_LL3, 130, 6); delay(400);
  moveTo(CH_LL2, cur_LL2, 85, 6);  delay(400);
  moveTo(CH_LL3, cur_LL3, 155, 6); delay(400);
  moveTo(CH_LL5, cur_LL5, 85, 6); moveTo(CH_RL5, cur_RL5, 85, 6); delay(500);
  moveTo(CH_LL5, cur_LL5, 75, 6); moveTo(CH_RL5, cur_RL5, 75, 6); delay(500);
  moveTo(CH_RL3, cur_RL3, 45, 6); delay(400);
  moveTo(CH_RL2, cur_RL2, 102, 6); delay(400);
  moveTo(CH_RL3, cur_RL3, 25, 6);  delay(400);
  moveTo(CH_LL5, cur_LL5, 85, 6); moveTo(CH_RL5, cur_RL5, 85, 6); delay(500);
}

// ─── Camera Stream Server ──────────────────────────────────────

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[Stream] Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    // Boundary
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    // Part header
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    // Image data
    res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }

  return res;
}

// ─── Camera Control Handler ────────────────────────────────────
static esp_err_t control_handler(httpd_req_t *req) {
  char buf[100];
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;

  if (buf_len > 1 && buf_len < sizeof(buf)) {
    httpd_req_get_url_query_str(req, buf, buf_len);

    char var[32], val[32];
    if (httpd_query_key_value(buf, "var", var, sizeof(var)) == ESP_OK &&
        httpd_query_key_value(buf, "val", val, sizeof(val)) == ESP_OK) {

      sensor_t * s = esp_camera_sensor_get();
      int value = atoi(val);

      if      (!strcmp(var, "framesize"))  s->set_framesize(s, (framesize_t)value);
      else if (!strcmp(var, "quality"))    s->set_quality(s, value);
      else if (!strcmp(var, "brightness")) s->set_brightness(s, value);
      else if (!strcmp(var, "contrast"))   s->set_contrast(s, value);
      else if (!strcmp(var, "sharpness"))  s->set_sharpness(s, value);
      else if (!strcmp(var, "awb"))        s->set_whitebal(s, value);
      else if (!strcmp(var, "aec"))        s->set_exposure_ctrl(s, value);
      else if (!strcmp(var, "agc"))        s->set_gain_ctrl(s, value);
    }
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_sendstr(req, "OK");
}

void startStreamServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t control_uri = {
    .uri       = "/control",
    .method    = HTTP_GET,
    .handler   = control_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &control_uri);
    Serial.println("[INFO] ✓ Stream server started on port 81");
  }
}

// ─── Serial Task ──────────────────────────────────────────────
void serialTask(void*) {
  for (;;) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if      (cmd == "i") initial_position();
      else if (cmd == "s") stand_straight();
      else if (cmd == "d") sit_down();
      else if (cmd == "u") stand_up();
      else if (cmd == "w") move_forward();
      else if (cmd == "r") turn_right();
      else if (cmd == "l") turn_left();
      else Serial.println("Unknown command");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ─── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  Wire.begin(21, 22);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(50);
  delay(100);

  initial_position();
  Serial.println("====== Robot Ready ======");

  // ─── Camera Config ───────────────────────────────────────
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk      = XCLK_GPIO_NUM;
  config.pin_pclk      = PCLK_GPIO_NUM;
  config.pin_vsync     = VSYNC_GPIO_NUM;
  config.pin_href      = HREF_GPIO_NUM;
  config.pin_sccb_sda  = SIOD_GPIO_NUM;
  config.pin_sccb_scl  = SIOC_GPIO_NUM;
  config.pin_pwdn      = PWDN_GPIO_NUM;
  config.pin_reset     = RESET_GPIO_NUM;
  config.xclk_freq_hz  = 20000000;
  config.pixel_format  = PIXFORMAT_JPEG;
  config.frame_size    = FRAMESIZE_QVGA;
  config.jpeg_quality  = 12;
  config.fb_count      = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
    return;
  }
  Serial.println("[INFO] ✓ Camera OK");

  // ─── WiFi ────────────────────────────────────────────────
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\n[INFO] ✓ WiFi connected");
  Serial.print("[INFO] Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream");

  // ─── Start Stream Server ─────────────────────────────────
  startStreamServer();

  // ─── Serial Task ─────────────────────────────────────────
  xTaskCreatePinnedToCore(serialTask, "serial_task", 4096, NULL, 1, NULL, 0);
}

// ─── Loop ─────────────────────────────────────────────────────
void loop() {
  delay(1000);
}