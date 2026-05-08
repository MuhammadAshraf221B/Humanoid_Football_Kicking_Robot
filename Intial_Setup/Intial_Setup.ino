#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 600

// ====== Channels ======
#define CH_hipL   0
#define CH_kneeL  1
#define CH_ankleL 2
#define CH_hipR   3
#define CH_kneeR  4
#define CH_ankleR 5

// ====== Offsets ======
#define hipLOffset   105
#define kneeLOffset  155
#define ankleLOffset 85
#define hipROffset   82
#define kneeROffset  25
#define ankleROffset 85

// ====== أبعاد الرجل (بتاعتك) ======
#define l1 5.5
#define l2 16.5
#define stepClearance 1
#define stepHeight 10

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setServo(int ch, int angle) {
  angle = constrain(angle, 0, 180);
  pca.setPWM(ch, 0, angleToPulse(angle));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(50);
  delay(500);

  setServo(CH_hipL,   hipLOffset);
  setServo(CH_kneeL,  kneeLOffset);
  setServo(CH_ankleL, ankleLOffset);
  setServo(CH_hipR,   hipROffset);
  setServo(CH_kneeR,  kneeROffset);
  setServo(CH_ankleR, ankleROffset);

  Serial.println("Robot Ready!");
  delay(2000);
}

void loop() {
}