#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>

static bool g_magOk = false;
static bool g_imuOk = false;
bool magReadRaw(int16_t *mx, int16_t *my, int16_t *mz);

const int LOOP_DELAY_MS = 100;
const int ERROR_TIME_OUT = 8000;
const float WHEEL_RADIUS_CM = 3.4f;
const int ENCODER_SLOTS_PER_WHEEL_REV = 20;
const float LEFT_GAIN = 1.0f;
const float RIGHT_GAIN = 1.0f;
const float WALL_DISTANCE_CM = 20.0f;
const float WALL_CLEAR_CM = 30.0f;

// Di thang: PWM co ban cho moi banh (chinh tay).
const int BASE_PWM = 85;

const int MIN_START_PWM_L = 60;
const int MIN_START_PWM_R = 60;

// PID mot vong: sai so = lech xung giua 2 banh (leftPulses - rightPulses), muc tieu = 0
float kpBal = 0.12f;
float kiBal = 0.002f;
float kdBal = 0.04f;
float iBal = 0.0f;
float prevBalErr = 0.0f;

/** false = di thang chi BASE_PWM doi xung (khong PID can bang) — bat truoc quay, tat sau quay */
static bool g_balancePidStraightEnabled = true;

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

/**
 * Sau quay 1 banh, mot banh nhieu xung mot banh it -> PID loi tong gap bi lech.
 * Reset xung ve 0 + xoa I/D state PID can bang de di thang lai giong luc xuat phat.
 */
void resetEncoderAndBalancePidForStraight() {
  noInterrupts();
  leftPulses = 0;
  rightPulses = 0;
  interrupts();
  iBal = 0.0f;
  prevBalErr = 0.0f;
}

/** Tat PID can bang truoc lenh quay: xoa I/D, khong con bu PWM lech tu doan di thang truoc */
void disableBalancePidBeforeTurn() {
  iBal = 0.0f;
  prevBalErr = 0.0f;
  g_balancePidStraightEnabled = false;
}

/** Bat lai PID sau quay + reset encoder/xung (goi cuoi pivot) */
void enableBalancePidAfterTurn() {
  resetEncoderAndBalancePidForStraight();
  g_balancePidStraightEnabled = true;
}

#pragma endregion

#pragma region CONFIG MPU6050 + Madgwick AHRS
const int MPU_SDA_PIN = 21;
const int MPU_SCL_PIN = 22;

MPU6050 mpu(MPU6050_DEFAULT_ADDRESS, &Wire);
Madgwick fusion;

/** Chu ky thich hop cho Madgwick.begin(fs): fs = 1/dt giua hai lan cap nhat (dt giay) */
static unsigned long prevFusionMicros = 0;

float yawDeg = 0.0f;

bool setupIMU() {
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000);
  mpu.initialize(ACCEL_FS::A2G, GYRO_FS::G250DPS);
  if (!mpu.testConnection()) {
    Serial.println(F("[MPU] Loi ket noi MPU6050"));
    g_imuOk = false;
    return false;
  }
  Serial.println(F("[MPU] Calibrate gyro (giu yen)..."));
  mpu.CalibrateGyro(6);
  Serial.println(F("[MPU] OK (Electronic Cats + Madgwick)"));
  prevFusionMicros = 0;
  g_imuOk = true;
  return true;
}

/**
 * Doc MPU6050 qua thu vien; fusion Madgwick (6 hoac 9 truc).
 * useMag: false khi quay pivot nhanh — tranh doc MMC9911 single-shot lau tre I2C.
 */
void updateYaw(bool useMag = true) {
  if (!g_imuOk) return;
  unsigned long nowUs = micros();
  if (prevFusionMicros == 0) {
    prevFusionMicros = nowUs;
    return;
  }
  float dt = (nowUs - prevFusionMicros) / 1000000.0f;
  prevFusionMicros = nowUs;
  if (dt <= 0.0f || dt > 0.25f) return;

  fusion.begin(1.0f / dt);

  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  const float ar = mpu.get_acce_resolution();
  const float gr = mpu.get_gyro_resolution();
  const float axg = ax * ar;
  const float ayg = ay * ar;
  const float azg = az * ar;
  const float gxd = gx * gr;
  const float gyd = gy * gr;
  const float gzd = gz * gr;

  int16_t mx = 0, my = 0, mz = 0;
  if (useMag && g_magOk && magReadRaw(&mx, &my, &mz)) {
    fusion.update(gxd, gyd, gzd, axg, ayg, azg, (float)mx, (float)my, (float)mz);
  } else {
    fusion.updateIMU(gxd, gyd, gzd, axg, ayg, azg);
  }

  yawDeg = fusion.getYaw();
}

#pragma endregion

#pragma region CONFIG MAG AK09911 (MCU-9911, cung bus I2C voi MPU6050)
const uint8_t MAG_ADDR = 0x0C;
const uint8_t AK09911_WIA1 = 0x48;
const uint8_t AK09911_WIA2 = 0x05;
const uint8_t AK09911_REG_ST1 = 0x10;
const uint8_t AK09911_REG_HXL = 0x11;
const uint8_t AK09911_REG_ST2 = 0x18;
const uint8_t AK09911_REG_CNTL2 = 0x31;
const uint8_t AK09911_REG_CNTL3 = 0x32;
const uint8_t AK09911_CNTL2_SINGLE = 0x01;
const uint8_t AK09911_CNTL2_POWER_DOWN = 0x00;

bool magReadRaw(int16_t *mx, int16_t *my, int16_t *mz) {
  if (!g_magOk || !mx || !my || !mz) return false;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(AK09911_REG_CNTL2);
  Wire.write(AK09911_CNTL2_SINGLE);
  Wire.endTransmission(true);

  uint8_t st1 = 0;
  for (int w = 0; w < 15; w++) {
    delay(1);
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(AK09911_REG_ST1);
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, (uint8_t)1);
    if (Wire.available() < 1) return false;
    st1 = Wire.read();
    if (st1 & 0x01) break;
  }
  if ((st1 & 0x01) == 0) return false;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(AK09911_REG_HXL);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return false;
  uint8_t hxl = Wire.read();
  uint8_t hxh = Wire.read();
  uint8_t hyl = Wire.read();
  uint8_t hyh = Wire.read();
  uint8_t hzl = Wire.read();
  uint8_t hzh = Wire.read();
  *mx = (int16_t)((hxh << 8) | hxl);
  *my = (int16_t)((hyh << 8) | hyl);
  *mz = (int16_t)((hzh << 8) | hzl);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(AK09911_REG_ST2);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)1);
  if (Wire.available() >= 1) {
    uint8_t st2 = Wire.read();
    if (st2 & 0x08) return false;
  }
  return true;
}

/**
 * Khoi tao AK09911C tren cung day SDA/SCL. Tra ve false neu khong dung chip / loi (MPU van chay).
 */
bool setupMagAK09911() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)2);
  if (Wire.available() < 2) {
    Serial.println(F("[MAG] Khong doc duoc WIA"));
    return false;
  }
  uint8_t w1 = Wire.read();
  uint8_t w2 = Wire.read();
  if (w1 != AK09911_WIA1 || w2 != AK09911_WIA2) {
    Serial.print(F("[MAG] WIA khong phai AK09911: "));
    Serial.print(w1, HEX);
    Serial.print(' ');
    Serial.println(w2, HEX);
    return false;
  }

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(AK09911_REG_CNTL3);
  Wire.write(0x01);
  Wire.endTransmission(true);
  delay(20);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(AK09911_REG_CNTL2);
  Wire.write(AK09911_CNTL2_POWER_DOWN);
  Wire.endTransmission(true);
  delay(2);

  g_magOk = true;
  Serial.println(F("[MAG] AK09911 OK (single-measure / read)"));
  return true;
}

#pragma endregion

#pragma region CONFIG CAM BIEN SIEU AM
const int TRIG_PIN = 33;
const int ECHO_PIN = 32;
const unsigned long DIST_READ_INTERVAL_MS = 80;

void setupUltrasonicPins() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long durationUs = pulseIn(ECHO_PIN, HIGH, 30000);
  if (durationUs == 0) {
    return -1.0f;
  }
  return (durationUs * 0.0343f) / 2.0f;
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
    int corr = 0;
    if (g_balancePidStraightEnabled) {
      corr = computeBalanceCorrection(dt);
    }
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

#pragma region QUAY_DAU_180_MOT_BANH_MPU
const int PIVOT_DRIVE_PWM = 95;
const int PIVOT_DRIVE_PWM_SLOW = 55;
const float PIVOT180_TARGET_DEG = 180.0f;
const float PIVOT180_TOL_DEG = 5.0f;
const float PIVOT180_SLOW_ZONE_DEG = 35.0f;
const unsigned long PIVOT180_TIMEOUT_MS = 25000;

/** Quay dau ben phai: giu banh phai, banh trai chay. Neu quay lui huong, doi thanh false */
const bool PIVOT_HEAD_RIGHT_LEFT_WHEEL_FORWARD = true;
/** Quay dau ben trai: giu banh trai, banh phai chay */
const bool PIVOT_HEAD_LEFT_RIGHT_WHEEL_FORWARD = true;

void stopAllMotorsOpenLoop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void applyMotorPower(int pwmL, int pwmR, bool forwardL, bool forwardR) {
  pwmL = constrain(pwmL, 0, 255);
  pwmR = constrain(pwmR, 0, 255);

  if (pwmL <= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } else {
    if (forwardL) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
    analogWrite(ENA, (int)((float)pwmL * LEFT_GAIN));
  }

  if (pwmR <= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  } else {
    if (forwardR) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    analogWrite(ENB, (int)((float)pwmR * RIGHT_GAIN));
  }
}

/**
 * Quay dau 180 do (mot banh lam chan, banh kia chay) — goc tu Madgwick (6DOF trong luc quay, nhanh).
 * pivotOnRightWheel = true: giu banh PHAI, banh TRAI chay (quay dau ben phai).
 * pivotOnRightWheel = false: giu banh TRAI, banh PHAI chay (quay dau ben trai).
 */
void pivot180OneWheel(bool pivotOnRightWheel) {
  disableBalancePidBeforeTurn();
  resetEncoderAndBalancePidForStraight();
  if (!g_imuOk) {
    // Fallback khi IMU loi: quay theo thoi gian de robot khong bi "dung im" qua lau.
    const unsigned long fallbackMs = 1400;
    unsigned long t0 = millis();
    while (millis() - t0 < fallbackMs) {
      if (pivotOnRightWheel) {
        applyMotorPower(PIVOT_DRIVE_PWM, 0, PIVOT_HEAD_RIGHT_LEFT_WHEEL_FORWARD, true);
      } else {
        applyMotorPower(0, PIVOT_DRIVE_PWM, true, PIVOT_HEAD_LEFT_RIGHT_WHEEL_FORWARD);
      }
      delay(5);
    }
    stopAllMotorsOpenLoop();
    enableBalancePidAfterTurn();
    delay(200);
    Serial.println(F("[PIVOT180] fallback theo thoi gian (IMU FAIL)"));
    return;
  }

  for (int i = 0; i < 15; i++) {
    updateYaw(false);
    delay(5);
  }

  float lastY = yawDeg;
  float accumTurnDeg = 0.0f;
  unsigned long t0 = millis();

  Serial.print(F("[PIVOT180] start "));
  Serial.println(pivotOnRightWheel ? F("hold RIGHT, drive LEFT") : F("hold LEFT, drive RIGHT"));

  while ((millis() - t0) < PIVOT180_TIMEOUT_MS) {
    updateYaw(false);

    float dy = yawDeg - lastY;
    if (dy > 180.0f) dy -= 360.0f;
    if (dy < -180.0f) dy += 360.0f;
    accumTurnDeg += fabsf(dy);
    lastY = yawDeg;

    int pwm = PIVOT_DRIVE_PWM;
    if (accumTurnDeg > PIVOT180_TARGET_DEG - PIVOT180_SLOW_ZONE_DEG) {
      pwm = PIVOT_DRIVE_PWM_SLOW;
    }

    if (pivotOnRightWheel) {
      applyMotorPower(pwm, 0, PIVOT_HEAD_RIGHT_LEFT_WHEEL_FORWARD, true);
    } else {
      applyMotorPower(0, pwm, true, PIVOT_HEAD_LEFT_RIGHT_WHEEL_FORWARD);
    }

    if (accumTurnDeg >= PIVOT180_TARGET_DEG - PIVOT180_TOL_DEG) {
      Serial.print(F("[PIVOT180] done accum="));
      Serial.println(accumTurnDeg, 1);
      break;
    }
    delay(5);
  }

  if ((millis() - t0) >= PIVOT180_TIMEOUT_MS) {
    Serial.println(F("[PIVOT180] TIMEOUT"));
  }

  stopAllMotorsOpenLoop();
  enableBalancePidAfterTurn();
  delay(300);
  if (g_magOk) {
    for (int i = 0; i < 10; i++) {
      updateYaw(true);
      delay(5);
    }
    Serial.print(F("[MAG] yaw Madgwick sau pivot (9DOF) = "));
    Serial.println(yawDeg, 1);
  }
}

#pragma endregion

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("[BOOT] start"));
  setupPINForEncoder();
  attachEncoderInterrupt();
  resetEncoderAndBalancePidForStraight();
  setupPINForL298N();
  setupUltrasonicPins();
  Serial.println(F("[BOOT] setup IMU"));
  const bool imuReady = setupIMU();
  if (!imuReady) {
    Serial.println(F("[BOOT] IMU fail -> van chay di thang, bo qua quay theo goc"));
  }
  Serial.println(F("[BOOT] setup MAG"));
  if (setupMagAK09911()) {
    for (int i = 0; i < 15; i++) {
      updateYaw(true);
      delay(5);
    }
    Serial.print(F("[MAG] yaw sau setup (9DOF) = "));
    Serial.println(yawDeg, 1);
  }
  Serial.println(F("[MODE] Di thang — gap tuong: lan 1 quay dau PHAI, lan 2 quay dau TRAI, xen ke..."));
}

/**
 * Di thang PID; doc sieu am moi DIST_READ_INTERVAL_MS.
 * Gap tuong: dung — quay dau 180 mot banh (xen ke phai/trai).
 * Chi bat lai phat hien tuong khi khoang cach > WALL_CLEAR_CM (tranh trigger nhieu lan).
 */
void loop() {
  static unsigned long lastDistMs = 0;
  static unsigned long lastHeartbeatMs = 0;
  static bool wallDetectArmed = true;
  /** Lan gap tuong tiep theo: true = quay dau ben phai (pivot180 giu banh phai), false = quay dau trai */
  static bool nextWallTurnPivotRight = true;

  updateYaw();

  unsigned long now = millis();
  if (now - lastHeartbeatMs >= 1000) {
    lastHeartbeatMs = now;
    Serial.print(F("[HB] imu="));
    Serial.print(g_imuOk ? F("OK") : F("FAIL"));
    Serial.print(F(" mag="));
    Serial.print(g_magOk ? F("OK") : F("FAIL"));
    Serial.print(F(" yaw="));
    Serial.println(yawDeg, 1);
  }
  if (now - lastDistMs >= DIST_READ_INTERVAL_MS) {
    lastDistMs = now;
    float d = readDistanceCm();

    if (wallDetectArmed && d > 0.0f && d <= WALL_DISTANCE_CM) {
      wallDetectArmed = false;
      stopAllMotorsOpenLoop();
      disableBalancePidBeforeTurn();
      Serial.print(F("[WALL] "));
      Serial.print(d, 1);
      Serial.println(F(" cm"));

      if (nextWallTurnPivotRight) {
        Serial.println(F("[TURN] Quay dau PHAI (giu banh phai, chay banh trai)"));
        pivot180OneWheel(true);
      } else {
        Serial.println(F("[TURN] Quay dau TRAI (giu banh trai, chay banh phai)"));
        pivot180OneWheel(false);
      }
      nextWallTurnPivotRight = !nextWallTurnPivotRight;
      delay(400);
      return;
    }

    if (!wallDetectArmed && (d < 0.0f || d >= WALL_CLEAR_CM)) {
      wallDetectArmed = true;
    }
  }

  executeControl(1, 1);
  delay(LOOP_DELAY_MS);
}
