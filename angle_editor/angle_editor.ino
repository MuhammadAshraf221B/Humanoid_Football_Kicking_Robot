#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 600

// ====== غيّر الـ channel والزاوية هنا ======
#define MY_CHANNEL 1
#define MY_ANGLE  140


int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(50);
  delay(100);

  Serial.print("Moving Channel ");
  Serial.print(MY_CHANNEL);
  Serial.print(" to ");
  Serial.println(MY_ANGLE);

  pca.setPWM(MY_CHANNEL, 0, angleToPulse(MY_ANGLE));

  Serial.println("Done!");
}

void loop() {}