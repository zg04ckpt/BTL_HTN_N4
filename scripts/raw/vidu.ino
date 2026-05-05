#include <Arduino.h>

const int LOOP_DELAY_MS = 20; // Delay mỗi loop
const int ERROR_TIME_OUT = 8000; // Thời gian tối đa thực hiện mỗi hành động trước khi về trạng thái lỗi
const float WHEEL_RADIUS_CM = 3.4f; // Bán kính bánh xe
const int ENCODER_SLOTS_PER_WHEEL_REV = 20; // Số xung trên 1 vòng quay
const float LEFT_GAIN = 1.0f; // Tỉ lệ lực bánh trái
const float RIGHT_GAIN = 0.8f; // Tỉ lệ lực bánh phải
// const int MIN_PWM = 80; // PWM tối thiểu để bánh quay
// const int MAX_PWM = 90; // PWM tối đa
const float VELOCITY = 1.5f; // Tốc độ mục tiêu = số xung / 100ms
const int MIN_START_PWM_L = 50; // PWM toi thieu de banh trai bat dau quay
const int MIN_START_PWM_R = 60; // PWM toi thieu de banh phai bat dau quay

#pragma region SETUP ENCODER
const int ENCODER_LEFT_PIN = 34;
const int ENCODER_RIGHT_PIN = 35;
volatile long leftPulses = 0;
volatile long rightPulses = 0;

void setupPINForEncoder() {
  pinMode(ENCODER_LEFT_PIN, INPUT);
  digitalWrite(ENCODER_LEFT_PIN, LOW);
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  digitalWrite(ENCODER_RIGHT_PIN, LOW);
}

void IRAM_ATTR incLeftPulses() { leftPulses++; }
void IRAM_ATTR incRightPulses() { rightPulses++; }

void attachEncoderInterrupt() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), incLeftPulses, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), incRightPulses, RISING);
}
#pragma endregion

#pragma region SETUP L298N
const int ENA = 13, IN1 = 12, IN2 = 14;
const int ENB = 25, IN3 = 27, IN4 = 26;
void setupPINForL298N() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

// PID trái
float kpL = 8.0f, kiL = 0.4f, kdL = 0.2f;
float iL = 0.0f, prevErrL = 0.0f, pwmLState = 0.0f;
// PID phải
float kpR = 8.0f, kiR = 0.4f, kdR = 0.2f;
float iR = 0.0f, prevErrR = 0.0f, pwmRState = 0.0f;

long prevLeftPulses = 0;
long prevRightPulses = 0;
unsigned long prevVelocityMs = 0;
float leftVelocityNow = 0.0f;   // xung / 100ms
float rightVelocityNow = 0.0f;  // xung / 100ms

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void updateWheelVelocity() {
  unsigned long now = millis();
  if (prevVelocityMs == 0) {
    prevVelocityMs = now;
    noInterrupts();
    prevLeftPulses = leftPulses;
    prevRightPulses = rightPulses;
    interrupts();
    return;
  }

  unsigned long dtMs = now - prevVelocityMs;
  if (dtMs < 20) return; // lấy mẫu đủ dày nhưng vẫn ổn định

  long lNow = 0, rNow = 0;
  noInterrupts();
  lNow = leftPulses;
  rNow = rightPulses;
  interrupts();

  long dL = lNow - prevLeftPulses;
  long dR = rNow - prevRightPulses;

  // Chuẩn hóa về đơn vị xung/100ms
  float scale = 100.0f / (float)dtMs;
  leftVelocityNow = (float)dL * scale;
  rightVelocityNow = (float)dR * scale;

  prevLeftPulses = lNow;
  prevRightPulses = rNow;
  prevVelocityMs = now;
}

// PID cho bánh trái: input là target velocity và velocity đo được (xung/100ms)
int computeLeftPwm(float targetVel, float measuredVel, float dt) {
  float error = targetVel - measuredVel; // error = VELOCITY muc tieu - velocity xung hien tai
  iL += error * dt;
  iL = clampFloat(iL, -80.0f, 80.0f); // chống tích lũy quá mức

  float d = (error - prevErrL) / dt;
  prevErrL = error;

  float delta = kpL * error + kiL * iL + kdL * d;
  pwmLState += delta;
  pwmLState = clampFloat(pwmLState, 0.0f, 255.0f);
  return (int)pwmLState;
}

// PID cho bánh phải: input là target velocity và velocity đo được (xung/100ms)
int computeRightPwm(float targetVel, float measuredVel, float dt) {
  float error = targetVel - measuredVel; // error = VELOCITY muc tieu - velocity xung hien tai
  iR += error * dt;
  iR = clampFloat(iR, -80.0f, 80.0f); // chống tích lũy quá mức

  float d = (error - prevErrR) / dt;
  prevErrR = error;

  float delta = kpR * error + kiR * iR + kdR * d;
  pwmRState += delta;
  pwmRState = clampFloat(pwmRState, 0.0f, 255.0f);
  return (int)pwmRState;
}

void executeControl(int speedL, int speedR) {
  updateWheelVelocity();

  unsigned long now = millis();
  static unsigned long prevPidMs = now;
  float dt = (now - prevPidMs) / 1000.0f;
  prevPidMs = now;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;

  // speedL/speedR chỉ dùng để xác định hướng chạy:
  // >0 tiến, <0 lùi, =0 dừng.
  int dirL = (speedL > 0) ? 1 : ((speedL < 0) ? -1 : 0);
  int dirR = (speedR > 0) ? 1 : ((speedR < 0) ? -1 : 0);

  float targetVelL = (dirL == 0) ? 0.0f : VELOCITY;
  float targetVelR = (dirR == 0) ? 0.0f : VELOCITY;

  int pwmL = (dirL == 0) ? 0 : computeLeftPwm(targetVelL, leftVelocityNow, dt);
  int pwmR = (dirR == 0) ? 0 : computeRightPwm(targetVelR, rightVelocityNow, dt);

  // Bu deadband khoi dong: moi banh co nguong PWM rieng.
  if (dirL != 0 && pwmL > 0 && pwmL < MIN_START_PWM_L) pwmL = MIN_START_PWM_L;
  if (dirR != 0 && pwmR > 0 && pwmR < MIN_START_PWM_R) pwmR = MIN_START_PWM_R;

  // Khi dừng thì reset trạng thái PID để lần chạy sau không bị "đọng" integral/pwm.
  if (dirL == 0) {
    iL = 0.0f;
    prevErrL = 0.0f;
    pwmLState = 0.0f;
  }
  if (dirR == 0) {
    iR = 0.0f;
    prevErrR = 0.0f;
    pwmRState = 0.0f;
  }

  if (dirL >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  if (dirR >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  analogWrite(ENA, constrain((int)(pwmL * LEFT_GAIN), 0, 255));
  analogWrite(ENB, constrain((int)(pwmR * RIGHT_GAIN), 0, 255));
}

#pragma endregion

void setup() {
  Serial.begin(115200);
  setupPINForEncoder();
  attachEncoderInterrupt();
  setupPINForL298N();
}

void loop() {
  // Demo: chạy thẳng bằng PID velocity.
  executeControl(1, 1);
  delay(LOOP_DELAY_MS);
}