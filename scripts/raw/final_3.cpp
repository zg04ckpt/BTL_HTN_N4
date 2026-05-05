#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/**
 * final_3.cpp
 * - Đi thẳng theo cm: encoder + PID cân hai bánh; điều kiện dừng |avg-target| + lệch / thoát lỏng.
 * - Quay tại chỗ theo xung: pulsesTurnInPlaceDeg + rotateInPlacePulsePID (nửa quãng đầu sàn PWM cao).
 */

const int LOOP_DELAY_MS = 20;
const int ERROR_TIME_OUT = 8000;

// --- Hình học bánh + encoder (20 lỗ / vòng bánh, 1 kênh đếm RISING = 20 xung/vòng) ---
const float WHEEL_RADIUS_CM = 3.4f;
const int ENCODER_SLOTS_PER_WHEEL_REV = 20;
/** Khoảng cách giữa hai tâm bánh (cm) — đo thực giữa tâm lăn. */
const float ROBOT_TRACK_WIDTH_CM = 12.7f;
/**
 * Bù quay encoder: thường thiếu góc do trượt/scrub -> tăng (>1).
 * Chỉ ảnh hưởng quay; không đụng WHEEL_RADIUS (đi thẳng theo cm).
 * Ví dụ đo được ~83° khi gọi 90°: thử ~90/83 ≈ 1.08.
 */
float ENC_ROT_INPLACE_SCALE = 1.8f;
float ENC_ROT_PIVOT_SCALE = 2.0f;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float wheelCircumferenceCm() { return 2.0f * (float)M_PI * WHEEL_RADIUS_CM; }

/** Xung trung bình (mỗi bánh) tương ứng quãng đường cm (hai bánh lăn như nhau). */
long cmToTargetAvgPulses(float cm) {
  float circ = wheelCircumferenceCm();
  if (circ <= 0.0f) return 0;
  float pulsesPerCm = (float)ENCODER_SLOTS_PER_WHEEL_REV / circ;
  long p = lroundf(cm * pulsesPerCm);
  if (p < 1 && cm > 0.0f) p = 1;
  return p;
}

float avgPulsesToCm(long avgPulses) {
  float circ = wheelCircumferenceCm();
  if (circ <= 0.0f) return 0.0f;
  return (float)avgPulses * circ / (float)ENCODER_SLOTS_PER_WHEEL_REV;
}

volatile long leftPulses = 0;
volatile long rightPulses = 0;

// --- L298N ---
const int ENA = 13, IN1 = 12, IN2 = 14;
const int ENB = 25, IN3 = 27, IN4 = 26;
// Trái đang đếm nhanh hơn phải trong log thực tế -> giảm LEFT_GAIN (tune tay thêm).
const float LEFT_GAIN = 0.95f;
const float RIGHT_GAIN = 1.0f;
const int MIN_PWM = 90;
const int MAX_PWM = 100;
/** Quay tại chỗ: nửa quãng xung đầu (≈ nửa góc) — sàn PWM cao để thắng ma sát tĩnh / motor yếu. */
const int ROT_INPLACE_PWM_FLOOR_FIRST_PHASE = 100;
/** Nửa cuối giảm lực quay để tránh quán tính vượt góc. */
const int ROT_INPLACE_PWM_FLOOR_SECOND_PHASE = 62;
const float ROT_INPLACE_PHASE_2_RATIO = 0.50f;
const float ROT_INPLACE_PHASE_2_POWER_SCALE = 0.65f;
// Quay MPU: gần đích giảm sàn PWM (tránh ép MIN_PWM=80 -> quán tính vượt góc).
const int MIN_ROT_PWM = 28;

void setupPINForL298N() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

/** minPwmFloor < 0: dùng MIN_PWM (đi thẳng). Ngược lại: sàn PWM khi quay gần đích. */
void executeControl(int speedL, int speedR, int minPwmFloor = -1) {
  if (minPwmFloor < 0) minPwmFloor = MIN_PWM;
  if (minPwmFloor > MAX_PWM) minPwmFloor = MAX_PWM;

  int leftCmd = speedL;
  int rightCmd = -speedR;

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

  if (leftCmd > 0 && leftCmd < minPwmFloor) leftCmd = minPwmFloor;
  if (rightCmd > 0 && rightCmd < minPwmFloor) rightCmd = minPwmFloor;

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

// --- Encoder ---
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

// --- MPU (chỉ dùng cho quay theo delta độ) ---
const uint8_t MPU_ADDR = 0x68;
const float GYRO_SCALE = 131.0f;
const float MPU_GYRO_DEADBAND_DPS = 0.35f;
const float MPU_RATE_LPF_ALPHA = 0.20f;
const float MPU_INTG_MIN_RATE_DPS = 0.08f;
const float MPU_RATE_MOTION_ON_DPS = 0.45f;
const float MPU_RATE_MOTION_OFF_DPS = 0.20f;
const uint8_t MPU_MOTION_CONFIRM_SAMPLES = 3;
float gyroOffsetZ = 0.0f;
unsigned long lastGyroMs = 0;
float yawDeg = 0.0f;
float filteredRateZ = 0.0f;
uint8_t mpuMotionSamples = 0;

void setupPinForMPU() {
  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void setupMPUVariableInitValues() {
  long sum = 0;
  Serial.println(F("[MPU] Calibrate Z gyro, keep still..."));
  for (int i = 0; i < 600; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)2);
    if (Wire.available() >= 2) {
      int16_t v = (int16_t)((Wire.read() << 8) | Wire.read());
      sum += v;
    }
    delay(3);
  }
  gyroOffsetZ = (float)sum / 600.0f;
  lastGyroMs = millis();
  yawDeg = 0.0f;
  filteredRateZ = 0.0f;
  mpuMotionSamples = 0;
  Serial.println(F("[MPU] OK"));
}

void updateYaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return;
  int16_t rawZ = (int16_t)((Wire.read() << 8) | Wire.read());

  unsigned long now = millis();
  float dt = (now - lastGyroMs) / 1000.0f;
  lastGyroMs = now;
  if (dt <= 0.0f || dt > 0.25f) return;

  float rateZ = (rawZ - gyroOffsetZ) / GYRO_SCALE;
  if (fabsf(rateZ) < MPU_GYRO_DEADBAND_DPS) rateZ = 0.0f; // bo rung nho quanh muc 0

  filteredRateZ += MPU_RATE_LPF_ALPHA * (rateZ - filteredRateZ); // low-pass 1 cuc
  float filtAbs = fabsf(filteredRateZ);
  if (filtAbs >= MPU_RATE_MOTION_ON_DPS) {
    if (mpuMotionSamples < MPU_MOTION_CONFIRM_SAMPLES) mpuMotionSamples++;
  } else if (filtAbs <= MPU_RATE_MOTION_OFF_DPS) {
    if (mpuMotionSamples > 0) mpuMotionSamples--;
  }

  const bool motionConfirmed = (mpuMotionSamples >= MPU_MOTION_CONFIRM_SAMPLES);
  if (motionConfirmed && filtAbs > MPU_INTG_MIN_RATE_DPS) yawDeg += filteredRateZ * dt;
}

float normalizeAngle180(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

// --- PID theo xung (encoder) ---
const long ENC_TURN_TARGET_PULSES = 120;
const long ENC_PULSE_DIST_TOL = 4;
const long ENC_PULSE_BAL_TOL = 5;
/** Khi TB xung đã sát target mà |dL-dR| vẫn lớn (lệch encoder / gain), vẫn thoát để tránh kẹt vòng lặp. */
const float ENC_STR_AVG_LOOSE_BAL_ERR = 22.0f;

const float ENC_DIST_KP = 0.45f;
const float ENC_BAL_KP = 0.42f;
const float ENC_BAL_KD = 0.10f;
const float ENC_BAL_KI = 0.035f;
const float ENC_BAL_I_CLAMP = 22.0f;
const float ENC_ADJ_MAX = 38.0f;

// Quay theo MPU (delta độ so với thời điểm bắt đầu)
const float YAW_ANGLE_TOL_DEG = 1.0f;
const float YAW_ROT_KP = 0.55f;
const float YAW_ROT_KD = 0.30f;
const int MAX_YAW_CORRECTION_SWINGS = 10;
const float YAW_PHASE_2_RATIO = 0.50f;     // 50% cuoi giam luc quay
const float YAW_PHASE_2_POWER_SCALE = 0.5f; // nua sau chi dung 50% luc
const float YAW_CORRECTION_SWING_MIN_ERR_DEG = 1.5f; // chi tinh overshoot khi vuot qua muc nay
const uint8_t YAW_STABLE_HIT_REQUIRED = 3;            // can on dinh lien tiep de ket thuc
const int YAW_PHASE_1_PWM_FLOOR = 58;
const int YAW_PHASE_2_PWM_FLOOR = 26;

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

// In mỗi vòng PID: xung tích, xung tăng trong vòng này, ước lượng xung/giây 2 bánh.
#ifndef DEBUG_ENC_PULSE_EACH_LOOP
#define DEBUG_ENC_PULSE_EACH_LOOP 1
#endif

namespace {
bool encDbgPrimed = false;
long encDbgPrevL = 0;
long encDbgPrevR = 0;
unsigned long encDbgPrevMs = 0;
uint32_t encDbgLoopIdx = 0;
} // namespace

void debugEncoderPulseRateInit() {
  encDbgPrimed = false;
  encDbgPrevMs = 0;
  encDbgLoopIdx = 0;
}

void debugEncoderPulseRateLine(const char *tag, long dL, long dR) {
#if DEBUG_ENC_PULSE_EACH_LOOP
  unsigned long now = millis();

  if (!encDbgPrimed) {
    encDbgPrevL = dL;
    encDbgPrevR = dR;
    encDbgPrevMs = now;
    encDbgPrimed = true;
    Serial.print(tag);
    Serial.println(" enc_debug: baseline set (next line = first d/loop & pps)");
    return;
  }

  long ddL = dL - encDbgPrevL;
  long ddR = dR - encDbgPrevR;
  unsigned long dtMs = now - encDbgPrevMs;
  if (dtMs == 0) dtMs = 1;
  float ppsL = 1000.0f * (float)ddL / (float)dtMs;
  float ppsR = 1000.0f * (float)ddR / (float)dtMs;

  Serial.print(tag);
  Serial.print(" #");
  Serial.print(encDbgLoopIdx++);
  Serial.print(" sum L/R=");
  Serial.print(dL);
  Serial.print("/");
  Serial.print(dR);
  Serial.print(" | d/loop L/R=");
  Serial.print(ddL);
  Serial.print("/");
  Serial.print(ddR);
  Serial.print(" | pps L/R=");
  Serial.print(ppsL, 0);
  Serial.print("/");
  Serial.print(ppsR, 0);
  Serial.print(" | diff_d=");
  Serial.println(ddL - ddR);

  encDbgPrevL = dL;
  encDbgPrevR = dR;
  encDbgPrevMs = now;
#else
  (void)tag;
  (void)dL;
  (void)dR;
#endif
}

void driveStraightPulsePID(long targetPulses) {
  encoderResetZero();
  debugEncoderPulseRateInit();
  unsigned long t0 = millis();
  unsigned long lastPidMs = millis();
  float lastBalErr = 0.0f;
  float balIntegral = 0.0f;

  Serial.print("[STR_PID] target avg pulses: ");
  Serial.println(targetPulses);

  while (true) {
    long dL = 0, dR = 0;
    encoderCountsSnapshot(dL, dR);
    debugEncoderPulseRateLine("[STR]", dL, dR);
    float avg = (float)(dL + dR) * 0.5f;
    float distErr = (float)targetPulses - avg;
    float balErr = (float)(dL - dR);
    float avgErrAbs = fabsf(distErr);

    // Trước đây: distErr <= tol (không fabs) -> overshoot âm vẫn thoát; và avg đúng nhưng |balErr|>tol -> kẹt vô hạn.
    const bool distOk = avgErrAbs <= (float)ENC_PULSE_DIST_TOL;
    const bool balOk = fabsf(balErr) <= (float)ENC_PULSE_BAL_TOL;
    const bool avgVeryClose = avgErrAbs <= 1.5f;
    const bool balLooseOk = fabsf(balErr) <= ENC_STR_AVG_LOOSE_BAL_ERR;

    if ((distOk && balOk) || (avgVeryClose && balLooseOk)) {
      stop();
      Serial.print("[STR_PID] done avg=");
      Serial.print(avg, 1);
      Serial.print(" bal=");
      Serial.print(balErr, 1);
      Serial.print(distOk && balOk ? F(" (dist+bal)") : F(" (avg tight, bal loose)"));
      Serial.println();
      return;
    }

    if ((int)(millis() - t0) > ERROR_TIME_OUT) {
      stop();
      Serial.println("[STR_PID] TIMEOUT");
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
    balIntegral += balErr * dt;
    if (balIntegral > ENC_BAL_I_CLAMP) balIntegral = ENC_BAL_I_CLAMP;
    if (balIntegral < -ENC_BAL_I_CLAMP) balIntegral = -ENC_BAL_I_CLAMP;

    float kpBal = ENC_BAL_KP;
    if (fabs(balErr) > 18.0f) kpBal *= 1.35f;

    float adj = kpBal * balErr + ENC_BAL_KD * dBal + ENC_BAL_KI * balIntegral;
    if (adj > ENC_ADJ_MAX) adj = ENC_ADJ_MAX;
    if (adj < -ENC_ADJ_MAX) adj = -ENC_ADJ_MAX;

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

void driveStraightCm(float cm) {
  long p = cmToTargetAvgPulses(cm);
  if (p <= 0) {
    Serial.println(F("[STR_CM] quang duong <= 0, bo qua"));
    return;
  }
  Serial.print(F("[STR_CM] "));
  Serial.print(cm, 2);
  Serial.print(F(" cm => xung TB moi banh "));
  Serial.print(p);
  Serial.print(F(" (~"));
  Serial.print(avgPulsesToCm(p), 2);
  Serial.println(F(" cm)"));
  driveStraightPulsePID(p);
}

/**
 * Quay tại chỗ: mỗi bánh lăn cung (W/2)·|θ|.
 * rotateInPlacePulsePID dùng progress ≈ (dL+dR)/2 ≈ xung một bánh khi cân.
 */
long pulsesTurnInPlaceDeg(float degAbs) {
  float rad = fabsf(degAbs) * (float)M_PI / 180.0f;
  float arcCm = (ROBOT_TRACK_WIDTH_CM * 0.5f) * rad * ENC_ROT_INPLACE_SCALE;
  return cmToTargetAvgPulses(arcCm);
}

/** Pivot quanh một bánh: bánh động lăn cung W·|θ|. */
long pulsesPivotMovingWheelDeg(float degAbs) {
  float rad = fabsf(degAbs) * (float)M_PI / 180.0f;
  float arcCm = ROBOT_TRACK_WIDTH_CM * rad * ENC_ROT_PIVOT_SCALE;
  return cmToTargetAvgPulses(arcCm);
}

/** Chỉ một bánh chạy; bánh kia PWM=0 (coast). pivotAboutLeft=true → tâm quay bánh trái, bánh phải chạy. */
void rotatePivotPulsePID(long targetPulsesMoving, bool pivotAboutLeft, bool ccw) {
  encoderResetZero();
  debugEncoderPulseRateInit();
  unsigned long t0 = millis();
  unsigned long lastPidMs = millis();

  Serial.print(F("[PIVOT_PID] target moving wheel pulses: "));
  Serial.print(targetPulsesMoving);
  Serial.print(F(" pivotLeft="));
  Serial.print(pivotAboutLeft ? 1 : 0);
  Serial.print(F(" ccw="));
  Serial.println(ccw ? 1 : 0);

  while (true) {
    long dL = 0, dR = 0;
    encoderCountsSnapshot(dL, dR);
    long dMove = pivotAboutLeft ? dR : dL;
    debugEncoderPulseRateLine("[PIV]", dL, dR);
    float distErr = (float)targetPulsesMoving - (float)dMove;

    if (distErr <= (float)ENC_PULSE_DIST_TOL) {
      stop();
      Serial.print(F("[PIVOT_PID] done dMove="));
      Serial.println(dMove);
      return;
    }

    if ((int)(millis() - t0) > ERROR_TIME_OUT) {
      stop();
      Serial.println(F("[PIVOT_PID] TIMEOUT"));
      return;
    }

    unsigned long now = millis();
    float dt = (now - lastPidMs) / 1000.0f;
    if (dt <= 0.0f || dt > 0.25f) dt = 0.02f;
    (void)dt;
    lastPidMs = now;

    float base = ENC_DIST_KP * distErr;
    if (base > (float)MAX_PWM) base = (float)MAX_PWM;
    if (base < (float)MIN_PWM) base = (float)MIN_PWM;

    int mag = (int)base;
    if (pivotAboutLeft) {
      int r = (int)((float)mag * RIGHT_GAIN);
      if (ccw)
        executeControl(0, r);
      else
        executeControl(0, -r);
    } else {
      int l = (int)((float)mag * LEFT_GAIN);
      if (ccw)
        executeControl(-l, 0);
      else
        executeControl(l, 0);
    }
    delay(LOOP_DELAY_MS);
  }
}

void rotateInPlacePulsePID(long targetPulsesHalfSum, bool ccw) {
  encoderResetZero();
  debugEncoderPulseRateInit();
  unsigned long t0 = millis();
  unsigned long lastPidMs = millis();
  float lastBalErr = 0.0f;
  float balIntegral = 0.0f;

  Serial.print("[ROT_PID] target (dL+dR)/2: ");
  Serial.println(targetPulsesHalfSum);

  while (true) {
    long dL = 0, dR = 0;
    encoderCountsSnapshot(dL, dR);
    debugEncoderPulseRateLine("[ROT]", dL, dR);
    float progress = (float)(dL + dR) * 0.5f;
    float distErr = (float)targetPulsesHalfSum - progress;
    float balErr = (float)(dL - dR);
    float progErrAbs = fabsf(distErr);

    const bool distOk = progErrAbs <= (float)ENC_PULSE_DIST_TOL;
    const bool balOk = fabsf(balErr) <= (float)ENC_PULSE_BAL_TOL;
    const bool progVeryClose = progErrAbs <= 1.5f;
    const bool balLooseOk = fabsf(balErr) <= ENC_STR_AVG_LOOSE_BAL_ERR;

    if ((distOk && balOk) || (progVeryClose && balLooseOk)) {
      stop();
      Serial.print("[ROT_PID] done progress=");
      Serial.print(progress, 1);
      Serial.print(" bal=");
      Serial.print(balErr, 1);
      Serial.print((distOk && balOk) ? F(" (dist+bal)") : F(" (prog tight, bal loose)"));
      Serial.println();
      return;
    }

    if ((int)(millis() - t0) > ERROR_TIME_OUT) {
      stop();
      Serial.println("[ROT_PID] TIMEOUT");
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
    balIntegral += balErr * dt;
    if (balIntegral > ENC_BAL_I_CLAMP) balIntegral = ENC_BAL_I_CLAMP;
    if (balIntegral < -ENC_BAL_I_CLAMP) balIntegral = -ENC_BAL_I_CLAMP;

    float kpBal = ENC_BAL_KP;
    if (fabs(balErr) > 18.0f) kpBal *= 1.35f;

    float adj = kpBal * balErr + ENC_BAL_KD * dBal + ENC_BAL_KI * balIntegral;
    if (adj > ENC_ADJ_MAX) adj = ENC_ADJ_MAX;
    if (adj < -ENC_ADJ_MAX) adj = -ENC_ADJ_MAX;

    const float phaseSwitchProgress = (float)targetPulsesHalfSum * ROT_INPLACE_PHASE_2_RATIO;
    const bool secondPhase = (targetPulsesHalfSum > 0L && progress >= phaseSwitchProgress);
    const float phasePowerScale = secondPhase ? ROT_INPLACE_PHASE_2_POWER_SCALE : 1.0f;
    float magL = (base - adj) * phasePowerScale;
    float magR = (base + adj) * phasePowerScale;
    const int rotFloor = secondPhase ? ROT_INPLACE_PWM_FLOOR_SECOND_PHASE : ROT_INPLACE_PWM_FLOOR_FIRST_PHASE;
    if (magL < (float)rotFloor) magL = (float)rotFloor;
    if (magR < (float)rotFloor) magR = (float)rotFloor;
    if (magL > (float)MAX_PWM) magL = (float)MAX_PWM;
    if (magR > (float)MAX_PWM) magR = (float)MAX_PWM;

    int lCmd = (int)(magL * LEFT_GAIN);
    int rCmd = (int)(magR * RIGHT_GAIN);
    if (ccw) {
      executeControl(-lCmd, rCmd, rotFloor);
    } else {
      executeControl(lCmd, -rCmd, rotFloor);
    }
    delay(LOOP_DELAY_MS);
  }
}

/**
 * Quay đúng delta độ (so với góc yaw lúc gọi hàm), đo bằng MPU.
 * Hai bánh vẫn PID cân theo lệch xung encoder (dL-dR) giống lúc quay xung.
 *
 * Nguyên nhân hay gây VƯỢT GÓC (overshoot):
 * 1) Ép PWM tối thiểu quá cao (MIN_PWM) khi |yawErr| đã nhỏ -> vẫn quay nhanh, quán tính trôi qua mục tiêu.
 * 2) Vòng lặp + delay: đọc MPU sau khi motor đã đẩy thêm một đoạn.
 * 3) Kp quay lớn / Kd nhỏ -> ít hãm ở cuối.
 * 4) stop() không phanh ngược; robot trượt nhẹ thêm sau khi cắt PWM.
 * Đã giảm (1) bằng sàn PWM động (MIN_ROT_PWM .. MIN_PWM) theo |yawErr|.
 * Tối đa MAX_YAW_CORRECTION_SWINGS lần đổi dấu yawErr (chỉnh qua mục tiêu) rồi dừng, tránh lắc lâu.
 */
void rotateByDeltaDegreesMPU(float deltaDeg) {
  updateYaw();
  const float goalYaw = yawDeg + deltaDeg;
  const float initialAbsErr = fabsf(deltaDeg);
  encoderResetZero();
  debugEncoderPulseRateInit();
  unsigned long t0 = millis();
  unsigned long lastPidMs = millis();
  float lastBalErr = 0.0f;
  float balIntegral = 0.0f;
  float lastYawErr = 0.0f;
  float progressPeak = 0.0f;
  bool yawPrimed = false;
  int yawCorrectionSwings = 0;
  uint8_t stableHits = 0;

  Serial.print(F("[ROT_MPU] delta "));
  Serial.print(deltaDeg, 1);
  Serial.println(F(" deg"));

  while (true) {
    updateYaw();
    float yawErr = normalizeAngle180(goalYaw - yawDeg);
    if (fabsf(yawErr) <= YAW_ANGLE_TOL_DEG) {
      stableHits++;
    } else {
      stableHits = 0;
    }
    if (stableHits >= YAW_STABLE_HIT_REQUIRED) {
      stop();
      Serial.print(F("[ROT_MPU] xong yaw="));
      Serial.println(yawDeg, 2);
      return;
    }
    if ((int)(millis() - t0) > ERROR_TIME_OUT) {
      stop();
      Serial.println(F("[ROT_MPU] TIMEOUT"));
      return;
    }

    long dL = 0, dR = 0;
    encoderCountsSnapshot(dL, dR);
    debugEncoderPulseRateLine("[YAW]", dL, dR);
    float balErr = (float)(dL - dR);

    unsigned long now = millis();
    float dt = (now - lastPidMs) / 1000.0f;
    if (dt <= 0.0f || dt > 0.25f) dt = 0.02f;
    lastPidMs = now;

    float dYawErr = 0.0f;
    if (yawPrimed) {
      dYawErr = (yawErr - lastYawErr) / dt;
      const bool crossedZero = (yawErr * lastYawErr < 0.0f);
      const bool meaningfulCross =
          (fabsf(yawErr) >= YAW_CORRECTION_SWING_MIN_ERR_DEG) ||
          (fabsf(lastYawErr) >= YAW_CORRECTION_SWING_MIN_ERR_DEG);
      if (crossedZero && meaningfulCross) {
        yawCorrectionSwings++;
        Serial.print(F("[ROT_MPU] chinh lai lan "));
        Serial.println(yawCorrectionSwings);
      }
    }
    yawPrimed = true;
    lastYawErr = yawErr;

    if (yawCorrectionSwings >= MAX_YAW_CORRECTION_SWINGS) {
      stop();
      Serial.print(F("[ROT_MPU] dung: toi da "));
      Serial.print(MAX_YAW_CORRECTION_SWINGS);
      Serial.print(F(" lan chinh, yaw="));
      Serial.println(yawDeg, 2);
      return;
    }

    float yawCmd = YAW_ROT_KP * yawErr + YAW_ROT_KD * dYawErr;
    float errAbs = fabs(yawErr);
    float progressRatio = 1.0f;
    if (initialAbsErr > 0.01f) {
      progressRatio = 1.0f - (errAbs / initialAbsErr); // 0 luc moi quay, 1 luc sap xong
      if (progressRatio < 0.0f) progressRatio = 0.0f;
      if (progressRatio > 1.0f) progressRatio = 1.0f;
    }
    if (progressRatio > progressPeak) progressPeak = progressRatio;
    progressRatio = progressPeak; // phase khong lui lai khi rung
    const bool secondPhase = progressRatio >= YAW_PHASE_2_RATIO;
    const float phasePowerScale = secondPhase ? YAW_PHASE_2_POWER_SCALE : 1.0f;
    int pwmFloor = secondPhase ? YAW_PHASE_2_PWM_FLOOR : YAW_PHASE_1_PWM_FLOOR;
    if (pwmFloor < 12) pwmFloor = 12; // tranh qua nho gay dung rung
    if (pwmFloor > MAX_PWM) pwmFloor = MAX_PWM;

    float turnMag = fabs(yawCmd) * phasePowerScale;
    if (turnMag < (float)pwmFloor) turnMag = (float)pwmFloor;
    if (turnMag > (float)MAX_PWM) turnMag = (float)MAX_PWM;

    float dBal = (balErr - lastBalErr) / dt;
    lastBalErr = balErr;
    balIntegral += balErr * dt;
    if (balIntegral > ENC_BAL_I_CLAMP) balIntegral = ENC_BAL_I_CLAMP;
    if (balIntegral < -ENC_BAL_I_CLAMP) balIntegral = -ENC_BAL_I_CLAMP;
    float kpBal = ENC_BAL_KP;
    if (fabs(balErr) > 18.0f) kpBal *= 1.35f;
    float adj = kpBal * balErr + ENC_BAL_KD * dBal + ENC_BAL_KI * balIntegral;
    if (adj > ENC_ADJ_MAX) adj = ENC_ADJ_MAX;
    if (adj < -ENC_ADJ_MAX) adj = -ENC_ADJ_MAX;

    float magL = turnMag - adj;
    float magR = turnMag + adj;
    if (magL < (float)pwmFloor) magL = (float)pwmFloor;
    if (magR < (float)pwmFloor) magR = (float)pwmFloor;
    if (magL > (float)MAX_PWM) magL = (float)MAX_PWM;
    if (magR > (float)MAX_PWM) magR = (float)MAX_PWM;

    int lP = (int)(magL * LEFT_GAIN);
    int rP = (int)(magR * RIGHT_GAIN);
    if (yawErr >= 0.0f) {
      executeControl(lP, -rP, pwmFloor);
    } else {
      executeControl(-lP, rP, pwmFloor);
    }
    delay(LOOP_DELAY_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  setupPINForL298N();
  setupPINForEncoder();
  attachEncoderInterrupt();
  setupPinForMPU();
  setupMPUVariableInitValues();
  stop();
  Serial.println(F("final_3: di thang (encoder) + quay tai cho (encoder)."));
}

void loop() {
  static bool done = false;
  if (!done) {
    delay(500);
    const float DEMO_CM = 20.0f;
    const float DEMO_TURN_DEG = 90.0f;

    Serial.println(F("[DEMO] test quay 90 do bang MPU"));
    // driveStraightCm(DEMO_CM);
    delay(400);
    rotateByDeltaDegreesMPU(DEMO_TURN_DEG); // +90 do CCW theo yaw MPU
    delay(400);
    rotateByDeltaDegreesMPU(-DEMO_TURN_DEG); // +90 do CCW theo yaw MPU

    // done = true;
    Serial.println(F("[DEMO] xong 1 vong."));
  }

  updateYaw();
  Serial.println(yawDeg, 3);
  delay(20);
}
