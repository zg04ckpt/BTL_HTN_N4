#include <Arduino.h>

/**
 * straight_no_pid_test.cpp
 * Test di thang KHONG dung PID/Gyro.
 * Muc tieu: kiem tra can bang co khi 2 banh bang toc do mo.
 */

// Pin map
const int ENA = 13, IN1 = 12, IN2 = 14;
const int ENB = 25, IN3 = 27, IN4 = 26;
const int ENCODER_LEFT_PIN = 34;
const int ENCODER_RIGHT_PIN = 35;

// Cau hinh toc do
const int MAX_PWM = 200;
const int TARGET_SPEED = 100;      // Toc do chay thu
const int RAMP_STEP = 1;          // Tang dan moi chu ky
const int LOOP_DELAY_MS = 20;
const int MOTOR_DEADZONE_PWM = 40;

// Can bang 2 ben (chinh tay neu robot lech)
const float LEFT_GAIN = 1.0f;
const float RIGHT_GAIN = 0.75f;

int currentSpeed = 0;
volatile long leftPulses = 0;
volatile long rightPulses = 0;

void IRAM_ATTR onLeftPulse() { leftPulses++; }
void IRAM_ATTR onRightPulse() { rightPulses++; }

void setupEncoderInputPin(uint8_t pin) {
  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);
}

void setMotors(int speedL, int speedR) {
  int leftCmd = (int)(speedL * LEFT_GAIN);
  int rightCmd = -(int)(speedR * RIGHT_GAIN); // Dao chieu banh phai

  if (leftCmd >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftCmd = -leftCmd;
  }

  if (rightCmd >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightCmd = -rightCmd;
  }

  if (leftCmd > 0 && leftCmd < MOTOR_DEADZONE_PWM) leftCmd = MOTOR_DEADZONE_PWM;
  if (rightCmd > 0 && rightCmd < MOTOR_DEADZONE_PWM) rightCmd = MOTOR_DEADZONE_PWM;

  analogWrite(ENA, constrain(leftCmd, 0, MAX_PWM));
  analogWrite(ENB, constrain(rightCmd, 0, MAX_PWM));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  setupEncoderInputPin(ENCODER_LEFT_PIN);
  setupEncoderInputPin(ENCODER_RIGHT_PIN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), onLeftPulse, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), onRightPulse, FALLING);

  stopMotors();
  delay(1000);
  Serial.println("Straight test (NO PID) start...");
  Serial.print("Encoder pins L/R: ");
  Serial.print(ENCODER_LEFT_PIN);
  Serial.print(" / ");
  Serial.println(ENCODER_RIGHT_PIN);
}

void loop() {
  if (currentSpeed < TARGET_SPEED) {
    currentSpeed += RAMP_STEP;
    if (currentSpeed > TARGET_SPEED) currentSpeed = TARGET_SPEED;
  }

  setMotors(currentSpeed, currentSpeed);

  static unsigned long lastLog = 0;
  static long prevLeft = 0;
  static long prevRight = 0;
  if (millis() - lastLog >= 300) {
    noInterrupts();
    long leftNow = leftPulses;
    long rightNow = rightPulses;
    interrupts();
    long dLeft = leftNow - prevLeft;
    long dRight = rightNow - prevRight;
    prevLeft = leftNow;
    prevRight = rightNow;
    long pulseDiff = dLeft - dRight;
    float diffPct = 0.0f;
    long maxDelta = max(abs(dLeft), abs(dRight));
    if (maxDelta > 0) diffPct = (100.0f * (float)pulseDiff) / (float)maxDelta;

    Serial.print("CMD L/R: ");
    Serial.print(currentSpeed);
    Serial.print(" / ");
    Serial.print(currentSpeed);
    Serial.print(" | Enc dL/dR: ");
    Serial.print(dLeft);
    Serial.print(" / ");
    Serial.print(dRight);
    Serial.print(" | Diff: ");
    Serial.print(pulseDiff);
    Serial.print(" (");
    Serial.print(diffPct, 1);
    Serial.println("%)");
    lastLog = millis();
  }

  delay(LOOP_DELAY_MS);
}
