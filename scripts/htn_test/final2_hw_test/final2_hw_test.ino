/**
 * Test phần cứng theo logic robot_scripts/final_2.cpp
 *
 * 1) (Tùy chọn) Lúc bật nguồn: đi thẳng theo xung encoder → quay tại chỗ (encoder PID).
 * 2) Sau đó: giống final_2 — cập nhật yaw gyro, in Serial, nếu trôi góc quá ngưỡng thì rotate(0).
 *
 * Chân: L298N, encoder 34/35, MPU I2C 21/22 — giữ nguyên final_2.
 *
 * Cấu hình: RUN_HW_TEST_ON_BOOT = 1 để chạy thử encoder một lần khi khởi động.
 * Arduino IDE: thư mục sketch phải trùng tên file .ino (final2_hw_test).
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// --- Bật = 1: sau setup MPU, chạy thử driveStraightPulsePID + rotateInPlacePulsePID rồi mới vào loop giám sát ---
#ifndef RUN_HW_TEST_ON_BOOT
#define RUN_HW_TEST_ON_BOOT 1
#endif

const int LOOP_DELAY_MS = 20;
const float WHEEL_R = 3.4f;
const float TOLERANCE = 1.0f;
const int ERROR_TIME_OUT = 5000;
volatile long leftPulses = 0;
volatile long rightPulses = 0;
volatile float yawDeg = 0;

#pragma region L298N + MOVING
const int ENA = 13, IN1 = 12, IN2 = 14;
const int ENB = 25, IN3 = 27, IN4 = 26;
const float LEFT_GAIN = 0.985f;
const float RIGHT_GAIN = 1.0f;
const int MIN_PWM = 80;
const int MAX_PWM = 100;

void setupPINForL298N() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void go(int speed) { executeControl(speed * LEFT_GAIN, speed * RIGHT_GAIN); }

void rotate(float targetDeg) {
  unsigned long startTime = millis();
  while (true) {
    updateYaw();

    float currentError = targetDeg - yawDeg;

    if (abs(currentError) <= TOLERANCE) {
      stop();
      return;
    }

    if (millis() - startTime > (unsigned long)ERROR_TIME_OUT) {
      stop();
      Serial.println(F("Rotate TIMEOUT!"));
      break;
    }

    int speed = 80;

    if (currentError > 0) {
      executeControl(-speed, speed);
    } else {
      executeControl(speed, -speed);
    }

    delay(100);
  }
}

void executeControl(int speedL, int speedR) {
  int leftCmd = speedL;
  int rightCmd = speedR;

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

  if (leftCmd > 0 && leftCmd < MIN_PWM) leftCmd = MIN_PWM;
  if (rightCmd > 0 && rightCmd < MIN_PWM) rightCmd = MIN_PWM;

  analogWrite(ENA, constrain(leftCmd, 0, MAX_PWM));
  analogWrite(ENB, constrain(rightCmd, 0, MAX_PWM));
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  delay(150);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
#pragma endregion

#pragma region MPU
const uint8_t MPU_ADDR = 0x68;
const float GYRO_SCALE = 131.0f;
float gyroOffsetZ = 0;
unsigned long lastGyroMs = 0;

void setupPinForMPU() {
  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void setupMPUVariableInitValues() {
  long sum = 0;
  Serial.println(F("Calibrating gyro Z — giu robot yen..."));

  for (int i = 0; i < 600; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)2);

    if (Wire.available() >= 2) {
      int16_t val = (Wire.read() << 8) | Wire.read();
      sum += val;
    }
    delay(3);
  }

  gyroOffsetZ = (float)sum / 600.0f;
  delay(100);
  lastGyroMs = millis();
  yawDeg = 0;
  Serial.println(F("Calibrate gyro done."));
}

void updateYaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);

  if (Wire.available() < 2) return;
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  unsigned long now = millis();
  float dt = (now - lastGyroMs) / 1000.0f;
  lastGyroMs = now;

  if (dt <= 0 || dt > 0.5f) return;

  float gyroRateZ = (rawZ - gyroOffsetZ) / GYRO_SCALE;

  if (abs(gyroRateZ) > 0.1) {
    yawDeg += gyroRateZ * dt;
  }
}

void debugMPU() {
  static unsigned long lastDebugMs = 0;

  if (millis() - lastDebugMs > 100) {
    lastDebugMs = millis();

    Serial.print(F("Yaw:"));
    Serial.print(yawDeg);
    Serial.print(F("\tBase:0"));
    Serial.println();
  }
}
#pragma endregion

#pragma region Encoder
const int ENCODER_LEFT_PIN = 34;
const int ENCODER_RIGHT_PIN = 35;

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

const long ENC_STRAIGHT_TARGET_PULSES = 400;
const long ENC_TURN_TARGET_PULSES = 120;
const long ENC_PULSE_DIST_TOL = 4;
const long ENC_PULSE_BAL_TOL = 5;

const float ENC_DIST_KP = 0.45f;
const float ENC_BAL_KP = 0.35f;
const float ENC_BAL_KD = 0.08f;

void encoderCountsSnapshot(long &outL, long &outR) {
  noInterrupts();
  outL = leftPulses;
  outR = rightPulses;
  interrupts();
}

void encoderResetZero() {
  noInterrupts();
  leftPulses = 0;
  rightPulses = 0;
  interrupts();
}

void driveStraightPulsePID(long targetPulses) {
  encoderResetZero();
  unsigned long t0 = millis();
  unsigned long lastPidMs = millis();
  float lastBalErr = 0.0f;

  Serial.print(F("[STR_ENC] target pulses (avg): "));
  Serial.println(targetPulses);

  while (true) {
    long dL = 0, dR = 0;
    encoderCountsSnapshot(dL, dR);
    float avg = (float)(dL + dR) * 0.5f;
    float distErr = (float)targetPulses - avg;
    float balErr = (float)(dL - dR);

    if (distErr <= (float)ENC_PULSE_DIST_TOL && fabs(balErr) <= (float)ENC_PULSE_BAL_TOL) {
      stop();
      Serial.print(F("[STR_ENC] done avg="));
      Serial.print(avg, 1);
      Serial.print(F(" bal="));
      Serial.println(balErr, 1);
      return;
    }

    if ((int)(millis() - t0) > ERROR_TIME_OUT) {
      stop();
      Serial.println(F("[STR_ENC] TIMEOUT"));
      return;
    }

    unsigned long now = millis();
    float dt = (now - lastPidMs) / 1000.0f;
    if (dt <= 0.0f || dt > 0.25f) dt = 0.02f;
    lastPidMs = now;

    float base = ENC_DIST_KP * distErr;
    if (base > (float)MAX_PWM) base = (float)MAX_PWM;
    if (base < (float)MIN_PWM) base = (float)MIN_PWM;

    float dBal = (balErr - lastBalErr) / dt;
    lastBalErr = balErr;
    float adj = ENC_BAL_KP * balErr + ENC_BAL_KD * dBal;
    if (adj > 25.0f) adj = 25.0f;
    if (adj < -25.0f) adj = -25.0f;

    float leftSpd = base - adj;
    float rightSpd = base + adj;
    if (leftSpd < (float)MIN_PWM) leftSpd = (float)MIN_PWM;
    if (rightSpd < (float)MIN_PWM) rightSpd = (float)MIN_PWM;
    if (leftSpd > (float)MAX_PWM) leftSpd = (float)MAX_PWM;
    if (rightSpd > (float)MAX_PWM) rightSpd = (float)MAX_PWM;

    executeControl((int)(leftSpd * LEFT_GAIN), (int)(rightSpd * RIGHT_GAIN));
    delay(LOOP_DELAY_MS);
  }
}

void rotateInPlacePulsePID(long targetPulsesHalfSum, bool ccw) {
  encoderResetZero();
  unsigned long t0 = millis();
  unsigned long lastPidMs = millis();
  float lastBalErr = 0.0f;

  Serial.print(F("[ROT_ENC] target (dL+dR)/2: "));
  Serial.println(targetPulsesHalfSum);

  while (true) {
    long dL = 0, dR = 0;
    encoderCountsSnapshot(dL, dR);
    float progress = (float)(dL + dR) * 0.5f;
    float distErr = (float)targetPulsesHalfSum - progress;
    float balErr = (float)(dL - dR);

    if (distErr <= (float)ENC_PULSE_DIST_TOL && fabs(balErr) <= (float)ENC_PULSE_BAL_TOL) {
      stop();
      Serial.print(F("[ROT_ENC] done progress="));
      Serial.print(progress, 1);
      Serial.print(F(" bal="));
      Serial.println(balErr, 1);
      return;
    }

    if ((int)(millis() - t0) > ERROR_TIME_OUT) {
      stop();
      Serial.println(F("[ROT_ENC] TIMEOUT"));
      return;
    }

    unsigned long now = millis();
    float dt = (now - lastPidMs) / 1000.0f;
    if (dt <= 0.0f || dt > 0.25f) dt = 0.02f;
    lastPidMs = now;

    float base = ENC_DIST_KP * distErr;
    if (base > (float)MAX_PWM) base = (float)MAX_PWM;
    if (base < (float)MIN_PWM) base = (float)MIN_PWM;

    float dBal = (balErr - lastBalErr) / dt;
    lastBalErr = balErr;
    float adj = ENC_BAL_KP * balErr + ENC_BAL_KD * dBal;
    if (adj > 25.0f) adj = 25.0f;
    if (adj < -25.0f) adj = -25.0f;

    float magL = base - adj;
    float magR = base + adj;
    if (magL < (float)MIN_PWM) magL = (float)MIN_PWM;
    if (magR < (float)MIN_PWM) magR = (float)MIN_PWM;
    if (magL > (float)MAX_PWM) magL = (float)MAX_PWM;
    if (magR > (float)MAX_PWM) magR = (float)MAX_PWM;

    int lCmd = (int)(magL * LEFT_GAIN);
    int rCmd = (int)(magR * RIGHT_GAIN);
    if (ccw) {
      executeControl(-lCmd, rCmd);
    } else {
      executeControl(lCmd, -rCmd);
    }
    delay(LOOP_DELAY_MS);
  }
}

#pragma endregion

/** Chuỗi test encoder giống final_2 (có thể tắt RUN_HW_TEST_ON_BOOT). */
static void runEncoderHardwareTest() {
  Serial.println(F("\n========== TEST ENCODER + PID (final_2) =========="));
  Serial.println(F("Buoc 1: di thang theo xung..."));
  delay(500);
  driveStraightPulsePID(ENC_STRAIGHT_TARGET_PULSES);

  delay(800);
  Serial.println(F("Buoc 2: quay tai cho CCW (encoder)..."));
  rotateInPlacePulsePID(ENC_TURN_TARGET_PULSES, true);

  delay(800);
  Serial.println(F("Buoc 3: quay tai cho CW (encoder)..."));
  rotateInPlacePulsePID(ENC_TURN_TARGET_PULSES, false);

  Serial.println(F("========== Ket thuc test encoder ==========\n"));
}

/** Zero lại góc gyro sau khi xe đã lăn — chỉ mang tính tham chiếu cho rotate(0). */
static void resetYawReferenceAfterMotion() {
  yawDeg = 0;
  lastGyroMs = millis();
}

void setup() {
  Serial.begin(115200);
  delay(400);

  setupPINForL298N();

  setupPINForEncoder();
  attachEncoderInterrupt();

  setupPinForMPU();
  setupMPUVariableInitValues();

#if RUN_HW_TEST_ON_BOOT
  Serial.println(F("Chuan bi test phan cung sau 3 giay — dat xe tren san rong, khong can tro."));
  delay(3000);
  runEncoderHardwareTest();
  resetYawReferenceAfterMotion();
  Serial.println(F("Da reset yaw (tham chieu). Neu gyro lech, loop se rotate(0)."));
#endif

  Serial.println(F("Loop: debug MPU + giu yaw ~ 0 (final_2). Go 't' de chay lai test encoder."));
}

void loop() {
  if (Serial.available()) {
    char c = (char)Serial.read();
    while (Serial.available()) {
      (void)Serial.read();
    }
    if (c == 't' || c == 'T') {
      runEncoderHardwareTest();
      resetYawReferenceAfterMotion();
    }
  }

  updateYaw();
  debugMPU();

  if (abs(yawDeg) > TOLERANCE) {
    rotate(0);
  }

  delay(LOOP_DELAY_MS);
}
