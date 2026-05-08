#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define CH_LL2 0
#define CH_LL3 1
#define CH_LL5 2
#define CH_RL2 3
#define CH_RL3 4
#define CH_RL5 5

#define SERVOMIN 150
#define SERVOMAX 600

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// ====== الزوايا الجديدة ======
int cur_LL2 = 105, cur_LL3 = 155, cur_LL5 = 85;
int cur_RL2 = 82,  cur_RL3 = 25,  cur_RL5 = 85;

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

void initial_position() {
  Serial.println(">> Initial Position");
  cur_LL2 = 105; cur_LL3 = 155; cur_LL5 = 85;
  cur_RL2 = 82;  cur_RL3 = 25;  cur_RL5 = 85;
  setServo(CH_LL2, 105);
  setServo(CH_LL3, 155);
  setServo(CH_LL5, 85);
  setServo(CH_RL2, 82);
  setServo(CH_RL3, 25);
  setServo(CH_RL5, 85);
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

  // ===== نقل الوزن لليمين =====
  moveTo(CH_LL5, cur_LL5, 95, 6);
  moveTo(CH_RL5, cur_RL5, 95, 6);
  delay(500);

  // ===== رفع الرجل الشمال =====
  moveTo(CH_LL3, cur_LL3, 130, 6);
  delay(400);
  // delay(3000);

  // ===== تقديم الرجل الشمال =====
  moveTo(CH_LL2, cur_LL2, 120, 6);
  delay(400);
  // delay(3000);
  // ===== تنزيل الرجل الشمال =====
  // moveTo(CH_LL3, cur_LL3, 120, 6);
  // // delay(400);
  // delay(3000);
  // ===== رجوع الوزن للنص =====
  moveTo(CH_LL5, cur_LL5, 85, 6);
  moveTo(CH_RL5, cur_RL5, 85, 6);
  delay(500);
  // delay(3000);
  // ===== نقل الوزن للشمال =====
  moveTo(CH_LL5, cur_LL5, 75, 6);
  moveTo(CH_RL5, cur_RL5, 75, 6);
  delay(500);
  // ===== رفع الرجل اليمين =====
  moveTo(CH_RL3, cur_RL3, 30, 6);
  delay(400);
  // ===== تقديم الرجل اليمين =====
  moveTo(CH_RL2, cur_RL2, 62, 6);
  delay(400);
  // ===== تنزيل الرجل اليمين =====
  moveTo(CH_RL3, cur_RL3, 25, 6);
  delay(400);
  // ===== رجوع للنص =====
  moveTo(CH_LL5, cur_LL5, 85, 6);
  moveTo(CH_RL5, cur_RL5, 85, 6);
  delay(500);
}

void turn_right() {
  Serial.println(">> Turn Right");
  moveTo(CH_RL5, cur_RL5, 60);
  moveTo(CH_LL5, cur_LL5, 60);
  delay(300);
  moveTo(CH_RL5, cur_RL5, 85);
  moveTo(CH_LL5, cur_LL5, 85);
}

void turn_left() {
  Serial.println(">> Turn Left");
  // ===== نقل الوزن لليمين =====
  moveTo(CH_LL5, cur_LL5, 95, 6);
  moveTo(CH_RL5, cur_RL5, 95, 6);
  delay(500);

  // ===== رفع الرجل الشمال =====
  moveTo(CH_LL3, cur_LL3, 130, 6);
  delay(400);

  // ===== تقديم الرجل الشمال =====
  moveTo(CH_LL2, cur_LL2, 85, 6);
  delay(400);

  // ===== تنزيل الرجل الشمال =====
  moveTo(CH_LL3, cur_LL3, 155, 6);
  delay(400);

  // ===== رجوع الوزن للنص =====
  moveTo(CH_LL5, cur_LL5, 85, 6);
  moveTo(CH_RL5, cur_RL5, 85, 6);
  delay(500);

  // ===== نقل الوزن للشمال =====
  moveTo(CH_LL5, cur_LL5, 75, 6);
  moveTo(CH_RL5, cur_RL5, 75, 6);
  delay(500);

  // ===== رفع الرجل اليمين =====
  moveTo(CH_RL3, cur_RL3, 45, 6);
  delay(400);

  // ===== تقديم الرجل اليمين =====
  moveTo(CH_RL2, cur_RL2, 102, 6);
  delay(400);

  // ===== تنزيل الرجل اليمين =====
  moveTo(CH_RL3, cur_RL3, 25, 6);
  delay(400);

  // ===== رجوع للنص =====
  moveTo(CH_LL5, cur_LL5, 85, 6);
  moveTo(CH_RL5, cur_RL5, 85, 6);
  delay(500);
}

void printCommands() {
  Serial.println("=== Commands ===");
  Serial.println("  i  → Initial Position");
  Serial.println("  s  → Stand");
  Serial.println("  d  → Sit Down");
  Serial.println("  u  → Stand Up");
  Serial.println("  w  → Move Forward");
  Serial.println("  r  → Turn Right");
  Serial.println("  l  → Turn Left");
  Serial.println("  LL2/LL3/LL5/RL2/RL3/RL5 [angle]");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(50);
  delay(100);

  initial_position();
  Serial.println("====== Robot Ready ======");
  printCommands();
}

void loop() {
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
        Serial.print(" → "); Serial.println(angle);
      } else {
        Serial.println("Unknown command!");
        printCommands();
      }
    }
  }
}