#include <Arduino.h>

const int LOOP_DELAY_MS = 100;
const int ERROR_TIME_OUT = 8000;
const float WHEEL_RADIUS_CM = 3.4f;
const int ENCODER_SLOTS_PER_WHEEL_REV = 20;
const float LEFT_GAIN = 1.0f;
const float RIGHT_GAIN = 1.0f;

// Di thang: PWM co ban cho moi banh (chinh tay).
const int BASE_PWM = 85;

const int MIN_START_PWM_L = 50;
const int MIN_START_PWM_R = 50;

// PID mot vong: sai so = lech xung giua 2 banh (leftPulses - rightPulses), muc tieu = 0
float kpBal = 0.12f;
float kiBal = 0.002f;
float kdBal = 0.04f;
float iBal = 0.0f;
float prevBalErr = 0.0f;

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

// ========== DEBUG TAM: PWM + lech xung/chu ky — dat SAU leftPulses/rightPulses (Arduino .ino cung quy tac)
// Xoa ca block nay khi khong can ==========
int g_dbgPwmL = 0;
int g_dbgPwmR = 0;
int g_dbgOutL = 0;
int g_dbgOutR = 0;

void debugPrintPwmAndPulseDeltaPerLoop() {
  static long prevLeft = 0;
  static long prevRight = 0;
  static bool primed = false;

  long lNow = 0;
  long rNow = 0;
  noInterrupts();
  lNow = leftPulses;
  rNow = rightPulses;
  interrupts();

  if (!primed) {
    prevLeft = lNow;
    prevRight = rNow;
    primed = true;
    return;
  }

  long dL = lNow - prevLeft;
  long dR = rNow - prevRight;
  long pulseDiffThisCycle = dL - dR;

  prevLeft = lNow;
  prevRight = rNow;

  Serial.print(F("[DBG_PWM] pwmL/R="));
  Serial.print(g_dbgPwmL);
  Serial.print(F("/"));
  Serial.print(g_dbgPwmR);
  Serial.print(F(" outL/R="));
  Serial.print(g_dbgOutL);
  Serial.print(F("/"));
  Serial.print(g_dbgOutR);
  Serial.print(F(" dL="));
  Serial.print(dL);
  Serial.print(F(" dR="));
  Serial.print(dR);
  Serial.print(F(" (dL-dR)="));
  Serial.println(pulseDiffThisCycle);
}
// ========== het DEBUG TAM ==========

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

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// PID can bang: err > 0 -> banh trai nhieu xung hon -> giam PWM trai, tang PWM phai
int computeBalanceCorrection(float dt) {
  long l = 0, r = 0;
  noInterrupts();
  l = leftPulses;
  r = rightPulses;
  interrupts();

  float err = (float)(l - r);
  iBal += err * dt;
  iBal = clampFloat(iBal, -3000.0f, 3000.0f);

  float d = (err - prevBalErr) / dt;
  prevBalErr = err;

  float out = kpBal * err + kiBal * iBal + kdBal * d;
  out = clampFloat(out, -50.0f, 50.0f);
  return (int)out;
}

void executeControl(int speedL, int speedR) {
  unsigned long now = millis();
  static unsigned long prevPidMs = now;
  float dt = (now - prevPidMs) / 1000.0f;
  prevPidMs = now;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;

  int dirL = (speedL > 0) ? 1 : ((speedL < 0) ? -1 : 0);
  int dirR = (speedR > 0) ? 1 : ((speedR < 0) ? -1 : 0);

  int pwmL = 0;
  int pwmR = 0;

  if (dirL != 0 && dirR != 0 && dirL == dirR) {
    int corr = computeBalanceCorrection(dt);
    pwmL = BASE_PWM - corr;
    pwmR = BASE_PWM + corr;
  } else {
    pwmL = (dirL == 0) ? 0 : BASE_PWM;
    pwmR = (dirR == 0) ? 0 : BASE_PWM;
    iBal = 0.0f;
    prevBalErr = 0.0f;
  }

  pwmL = constrain(pwmL, 0, 255);
  pwmR = constrain(pwmR, 0, 255);

  if (dirL != 0 && pwmL > 0 && pwmL < MIN_START_PWM_L) pwmL = MIN_START_PWM_L;
  if (dirR != 0 && pwmR > 0 && pwmR < MIN_START_PWM_R) pwmR = MIN_START_PWM_R;

  if (dirL == 0) {
    iBal = 0.0f;
    prevBalErr = 0.0f;
  }
  if (dirR == 0) {
    iBal = 0.0f;
    prevBalErr = 0.0f;
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

  g_dbgPwmL = pwmL;
  g_dbgPwmR = pwmR;
  g_dbgOutL = constrain((int)((float)pwmL * LEFT_GAIN), 0, 255);
  g_dbgOutR = constrain((int)((float)pwmR * RIGHT_GAIN), 0, 255);
  analogWrite(ENA, g_dbgOutL);
  analogWrite(ENB, g_dbgOutR);
}

#pragma endregion

void setup() {
  Serial.begin(115200);
  setupPINForEncoder();
  attachEncoderInterrupt();
  setupPINForL298N();
}

void loop() {
  executeControl(1, 1);
  debugPrintPwmAndPulseDeltaPerLoop();
  delay(LOOP_DELAY_MS);
}
