#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>

// ===================== I2C / IMU / MAG =====================
const int MPU_SDA_PIN = 21;
const int MPU_SCL_PIN = 22;

const uint8_t AK09911_ADDR = 0x0C;   // AK09911 / MCU-9911 (AKM)
const uint8_t QMC5883_ADDR = 0x0D;   // QMC5883L — nhiều board “9911” thực tế là QMC
const uint8_t AK09911_WIA1 = 0x48;
const uint8_t AK09911_WIA2 = 0x05;
const uint8_t AK09911_REG_ST1 = 0x10;
const uint8_t AK09911_REG_HXL = 0x11;
const uint8_t AK09911_REG_ST2 = 0x18;
const uint8_t AK09911_REG_CNTL2 = 0x31;
const uint8_t AK09911_REG_CNTL3 = 0x32;
const uint8_t AK09911_CNTL2_SINGLE = 0x01;
const uint8_t AK09911_CNTL2_POWER_DOWN = 0x00;

MPU6050 mpu(0x68, &Wire);
Madgwick fusion;

bool g_imuOk = false;
bool g_magOk = false;
enum { MAG_CHIP_NONE = 0, MAG_CHIP_AK09911, MAG_CHIP_QMC5883 };
static uint8_t g_magChip = MAG_CHIP_NONE;
static uint8_t g_magAddr = 0;
unsigned long prevFusionUs = 0;
unsigned long g_updateCount = 0;
unsigned long g_magReadFailCount = 0;

// ===================== MOTOR (L298N) =====================
const int ENA = 13;
const int IN1 = 12;
const int IN2 = 14;
const int ENB = 25;
const int IN3 = 27;
const int IN4 = 26;
const int LED_DEBUG_PIN = 2;   // ESP32 devkit thuong la LED onboard

// ===================== TEST LOGIC =====================
float yawDeg = 0.0f;
float baseHeadingDeg = 0.0f;

const float TOLERANCE_DEG = 2.0f;
const unsigned long ERROR_TIME_OUT = 5000;
const float LEFT_GAIN = 0.985f;
const float RIGHT_GAIN = 1.0f;
const int MIN_PWM = 70;
const int MAX_PWM = 105;

const unsigned long LOG_INTERVAL_MS = 400;
unsigned long lastLogMs = 0;
unsigned long lastLedBeatMs = 0;

void blinkDebug(int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_DEBUG_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_DEBUG_PIN, LOW);
    delay(offMs);
  }
}

float normalizeAngle180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

int i2cProbeAddr(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true);  // 0 = ACK
}

int readReg8(uint8_t addr, uint8_t reg, uint8_t *out) {
  if (!out) return -99;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  int tx = Wire.endTransmission(false);
  if (tx != 0) return tx;
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available() < 1) return -98;
  *out = Wire.read();
  return 0;
}

void scanI2CBus() {
  Serial.println(F("[I2C] scanning 0x03..0x77"));
  int count = 0;
  for (uint8_t a = 0x03; a <= 0x77; a++) {
    int e = i2cProbeAddr(a);
    if (e == 0) {
      Serial.print(F("[I2C] found 0x"));
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
      count++;
    }
    delay(1);
  }
  Serial.print(F("[I2C] device count="));
  Serial.println(count);
}

bool magReadRawAK09911(int16_t *mx, int16_t *my, int16_t *mz) {
  if (!mx || !my || !mz) return false;

  Wire.beginTransmission(g_magAddr);
  Wire.write(AK09911_REG_CNTL2);
  Wire.write(AK09911_CNTL2_SINGLE);
  int tx = Wire.endTransmission(true);
  if (tx != 0) {
    g_magReadFailCount++;
    if ((g_magReadFailCount % 20UL) == 1UL) {
      Serial.print(F("[MAG][AK][ERR] CNTL2 tx fail code="));
      Serial.println(tx);
    }
    return false;
  }

  uint8_t st1 = 0;
  for (int i = 0; i < 15; i++) {
    delay(1);
    Wire.beginTransmission(g_magAddr);
    Wire.write(AK09911_REG_ST1);
    tx = Wire.endTransmission(false);
    if (tx != 0) {
      g_magReadFailCount++;
      if ((g_magReadFailCount % 20UL) == 1UL) {
        Serial.print(F("[MAG][AK][ERR] ST1 tx fail code="));
        Serial.println(tx);
      }
      return false;
    }
    Wire.requestFrom(g_magAddr, (uint8_t)1);
    if (Wire.available() < 1) {
      g_magReadFailCount++;
      if ((g_magReadFailCount % 20UL) == 1UL) {
        Serial.println(F("[MAG][AK][ERR] ST1 no data"));
      }
      return false;
    }
    st1 = Wire.read();
    if (st1 & 0x01) break;
  }
  if ((st1 & 0x01) == 0) {
    g_magReadFailCount++;
    if ((g_magReadFailCount % 20UL) == 1UL) {
      Serial.println(F("[MAG][AK][ERR] DRDY timeout"));
    }
    return false;
  }

  Wire.beginTransmission(g_magAddr);
  Wire.write(AK09911_REG_HXL);
  tx = Wire.endTransmission(false);
  if (tx != 0) {
    g_magReadFailCount++;
    if ((g_magReadFailCount % 20UL) == 1UL) {
      Serial.print(F("[MAG][AK][ERR] HXL tx fail code="));
      Serial.println(tx);
    }
    return false;
  }
  Wire.requestFrom(g_magAddr, (uint8_t)6);
  if (Wire.available() < 6) {
    g_magReadFailCount++;
    if ((g_magReadFailCount % 20UL) == 1UL) {
      Serial.print(F("[MAG][AK][ERR] XYZ short read n="));
      Serial.println(Wire.available());
    }
    return false;
  }

  uint8_t hxl = Wire.read();
  uint8_t hxh = Wire.read();
  uint8_t hyl = Wire.read();
  uint8_t hyh = Wire.read();
  uint8_t hzl = Wire.read();
  uint8_t hzh = Wire.read();

  *mx = (int16_t)((hxh << 8) | hxl);
  *my = (int16_t)((hyh << 8) | hyl);
  *mz = (int16_t)((hzh << 8) | hzl);

  Wire.beginTransmission(g_magAddr);
  Wire.write(AK09911_REG_ST2);
  tx = Wire.endTransmission(false);
  if (tx != 0) {
    g_magReadFailCount++;
    if ((g_magReadFailCount % 20UL) == 1UL) {
      Serial.print(F("[MAG][AK][ERR] ST2 tx fail code="));
      Serial.println(tx);
    }
    return false;
  }
  Wire.requestFrom(g_magAddr, (uint8_t)1);
  if (Wire.available() >= 1) {
    uint8_t st2 = Wire.read();
    if (st2 & 0x08) {
      g_magReadFailCount++;
      if ((g_magReadFailCount % 20UL) == 1UL) {
        Serial.println(F("[MAG][AK][ERR] overflow HOFL=1"));
      }
      return false;
    }
  }
  return true;
}

bool magReadRawQMC5883(int16_t *mx, int16_t *my, int16_t *mz) {
  if (!mx || !my || !mz) return false;

  uint8_t st = 0;
  if (readReg8(g_magAddr, 0x06, &st) != 0) {
    g_magReadFailCount++;
    return false;
  }
  if ((st & 0x01) == 0) {
    g_magReadFailCount++;
    if ((g_magReadFailCount % 40UL) == 1UL) {
      Serial.println(F("[MAG][QMC][WARN] DRDY=0 (bo qua lan nay)"));
    }
    return false;
  }

  Wire.beginTransmission(g_magAddr);
  Wire.write(0x00);
  int tx = Wire.endTransmission(false);
  if (tx != 0) {
    g_magReadFailCount++;
    if ((g_magReadFailCount % 20UL) == 1UL) {
      Serial.print(F("[MAG][QMC][ERR] data ptr tx fail code="));
      Serial.println(tx);
    }
    return false;
  }
  uint8_t n = Wire.requestFrom(g_magAddr, (uint8_t)6);
  if (n != 6 || Wire.available() < 6) {
    g_magReadFailCount++;
    if ((g_magReadFailCount % 20UL) == 1UL) {
      Serial.print(F("[MAG][QMC][ERR] short read n="));
      Serial.println(Wire.available());
    }
    return false;
  }

  uint8_t xl = Wire.read();
  uint8_t xh = Wire.read();
  uint8_t yl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t zl = Wire.read();
  uint8_t zh = Wire.read();
  *mx = (int16_t)((int16_t)xh << 8 | xl);
  *my = (int16_t)((int16_t)yh << 8 | yl);
  *mz = (int16_t)((int16_t)zh << 8 | zl);
  return true;
}

bool magReadRaw(int16_t *mx, int16_t *my, int16_t *mz) {
  if (!g_magOk || !mx || !my || !mz) return false;
  if (g_magChip == MAG_CHIP_QMC5883) return magReadRawQMC5883(mx, my, mz);
  if (g_magChip == MAG_CHIP_AK09911) return magReadRawAK09911(mx, my, mz);
  return false;
}

bool setupMagAK09911() {
  Serial.println(F("[MAG] probing AK09911 @0x0C..."));
  Wire.beginTransmission(AK09911_ADDR);
  Wire.write(0x00);
  int tx = Wire.endTransmission(false);
  if (tx != 0) {
    Serial.print(F("[MAG][AK][ERR] probe tx fail code="));
    Serial.println(tx);
    return false;
  }
  Wire.requestFrom(AK09911_ADDR, (uint8_t)2);
  if (Wire.available() < 2) {
    Serial.println(F("[MAG][AK][ERR] probe no WIA"));
    return false;
  }
  uint8_t w1 = Wire.read();
  uint8_t w2 = Wire.read();
  Serial.print(F("[MAG][AK] WIA="));
  Serial.print(w1, HEX);
  Serial.print(' ');
  Serial.println(w2, HEX);
  if (w1 != AK09911_WIA1 || w2 != AK09911_WIA2) {
    Serial.println(F("[MAG][AK][ERR] wrong chip id"));
    return false;
  }

  Wire.beginTransmission(AK09911_ADDR);
  Wire.write(AK09911_REG_CNTL3);
  Wire.write(0x01);
  tx = Wire.endTransmission(true);
  if (tx != 0) {
    Serial.print(F("[MAG][AK][ERR] reset tx fail code="));
    Serial.println(tx);
    return false;
  }
  delay(20);

  Wire.beginTransmission(AK09911_ADDR);
  Wire.write(AK09911_REG_CNTL2);
  Wire.write(AK09911_CNTL2_POWER_DOWN);
  tx = Wire.endTransmission(true);
  if (tx != 0) {
    Serial.print(F("[MAG][AK][ERR] powerdown tx fail code="));
    Serial.println(tx);
    return false;
  }

  Serial.println(F("[MAG][AK] init OK"));
  return true;
}

bool setupMagQMC5883() {
  Serial.println(F("[MAG] probing QMC5883 @0x0D..."));
  if (i2cProbeAddr(QMC5883_ADDR) != 0) {
    Serial.println(F("[MAG][QMC][ERR] no ACK"));
    return false;
  }

  Wire.beginTransmission(QMC5883_ADDR);
  Wire.write(0x0B);
  Wire.write(0x01);
  if (Wire.endTransmission(true) != 0) {
    Serial.println(F("[MAG][QMC][ERR] SET/RESET write fail"));
    return false;
  }
  delay(10);

  Wire.beginTransmission(QMC5883_ADDR);
  Wire.write(0x09);
  Wire.write(0x0D);
  if (Wire.endTransmission(true) != 0) {
    Serial.println(F("[MAG][QMC][ERR] CTRL1 write fail"));
    return false;
  }
  delay(10);

  Serial.println(F("[MAG][QMC] init OK (continuous)"));
  return true;
}

bool setupMagAuto() {
  g_magOk = false;
  g_magChip = MAG_CHIP_NONE;
  g_magAddr = 0;

  if (i2cProbeAddr(AK09911_ADDR) == 0) {
    if (setupMagAK09911()) {
      g_magChip = MAG_CHIP_AK09911;
      g_magAddr = AK09911_ADDR;
      g_magOk = true;
      Serial.println(F("[MAG] dung: AK09911 @0x0C"));
      return true;
    }
  }

  if (i2cProbeAddr(QMC5883_ADDR) == 0) {
    if (setupMagQMC5883()) {
      g_magChip = MAG_CHIP_QMC5883;
      g_magAddr = QMC5883_ADDR;
      g_magOk = true;
      Serial.println(F("[MAG] dung: QMC5883L @0x0D (khong phai AK09911 0x0C)"));
      return true;
    }
  }

  Serial.println(F("[MAG][ERR] Khong nhan la ban @0x0C (AK) hoac @0x0D (QMC)"));
  return false;
}

bool setupIMU() {
  Serial.println(F("[MPU] probing 0x68/0x69..."));
  const uint8_t candidates[2] = {0x68, 0x69};
  uint8_t foundAddr = 0;
  for (int i = 0; i < 2; i++) {
    int e = i2cProbeAddr(candidates[i]);
    Serial.print(F("[MPU] probe 0x"));
    Serial.print(candidates[i], HEX);
    Serial.print(F(" -> "));
    Serial.println(e == 0 ? F("ACK") : F("NACK"));
    if (e == 0 && foundAddr == 0) foundAddr = candidates[i];
  }

  if (foundAddr == 0) {
    Serial.println(F("[MPU][ERR] khong thay device 0x68/0x69"));
    return false;
  }

  uint8_t who = 0xFF;
  int rw = readReg8(foundAddr, 0x75, &who);  // WHO_AM_I
  Serial.print(F("[MPU] WHO_AM_I read="));
  if (rw == 0) {
    Serial.println(who, HEX);
  } else {
    Serial.print(F("FAIL code="));
    Serial.println(rw);
  }

  mpu = MPU6050(foundAddr, &Wire);
  Serial.print(F("[MPU] initialize at 0x"));
  Serial.println(foundAddr, HEX);
  mpu.initialize(ACCEL_FS::A2G, GYRO_FS::G250DPS);

  // Thu vien nay co the testConnection() khat khe voi clone -> cho phep tiep tuc neu WHO_AM_I hop ly
  bool testOk = mpu.testConnection();
  bool whoLooksMpu = (rw == 0) && ((who & 0x7E) == 0x68 || who == 0x68 || who == 0x69);
  Serial.print(F("[MPU] testConnection="));
  Serial.println(testOk ? F("OK") : F("FAIL"));
  if (!testOk && !whoLooksMpu) {
    Serial.println(F("[MPU][ERR] testConnection FAIL + WHO_AM_I khong hop ly"));
    return false;
  }
  if (!testOk && whoLooksMpu) {
    Serial.println(F("[MPU][WARN] testConnection FAIL, nhung WHO_AM_I hop ly -> tiep tuc"));
  }

  Serial.println(F("[MPU] calibrating gyro..."));
  mpu.CalibrateGyro(6);
  Serial.println(F("[MPU] calibration done"));
  prevFusionUs = 0;
  return true;
}

void updateYaw(bool useMag) {
  if (!g_imuOk) return;

  unsigned long nowUs = micros();
  if (prevFusionUs == 0) {
    prevFusionUs = nowUs;
    return;
  }
  float dt = (nowUs - prevFusionUs) / 1000000.0f;
  prevFusionUs = nowUs;
  if (dt <= 0.0f || dt > 0.25f) return;

  fusion.begin(1.0f / dt);

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  const float ar = mpu.get_acce_resolution();  // g/LSB
  const float gr = mpu.get_gyro_resolution();  // dps/LSB

  float axg = ax * ar;
  float ayg = ay * ar;
  float azg = az * ar;
  float gxd = gx * gr;
  float gyd = gy * gr;
  float gzd = gz * gr;

  int16_t mx = 0, my = 0, mz = 0;
  if (useMag && g_magOk && magReadRaw(&mx, &my, &mz)) {
    fusion.update(gxd, gyd, gzd, axg, ayg, azg, (float)mx, (float)my, (float)mz);
  } else {
    fusion.updateIMU(gxd, gyd, gzd, axg, ayg, azg);
  }

  yawDeg = fusion.getYaw();
  g_updateCount++;
}

void setupMotorPins() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
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

void stopBrakeLikeFinal2() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  delay(120);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void rotateToAngle(float targetDeg) {
  unsigned long startTime = millis();
  Serial.print(F("[ROT] start target="));
  Serial.println(targetDeg, 2);
  while (true) {
    updateYaw(true);
    float currentError = normalizeAngle180(targetDeg - yawDeg);
    float absErr = fabsf(currentError);

    if (absErr <= TOLERANCE_DEG) {
      stopBrakeLikeFinal2();
      Serial.print(F("[ROT] done yaw="));
      Serial.print(yawDeg, 2);
      Serial.print(F(" err="));
      Serial.println(currentError, 2);
      return;
    }

    if (millis() - startTime > ERROR_TIME_OUT) {
      stopBrakeLikeFinal2();
      Serial.println(F("[ROT] TIMEOUT"));
      Serial.print(F("[ROT] timeout yaw="));
      Serial.print(yawDeg, 2);
      Serial.print(F(" target="));
      Serial.print(targetDeg, 2);
      Serial.print(F(" err="));
      Serial.println(currentError, 2);
      return;
    }

    int speed = 90;
    if (absErr < 15.0f) speed = 75;
    if (absErr < 7.0f) speed = 68;

    int lCmd = (int)(speed * LEFT_GAIN);
    int rCmd = (int)(speed * RIGHT_GAIN);
    if (currentError > 0.0f) {
      // Quay phai: trai lui, phai tien (giong final_2)
      executeControl(-lCmd, rCmd);
    } else {
      // Quay trai
      executeControl(lCmd, -rCmd);
    }
    delay(12);
  }
}

float captureBaseHeading() {
  // Lay trung binh circular de giam nhiu luc khoi dong
  const int N = 80;
  float sx = 0.0f;
  float sy = 0.0f;
  int ok = 0;
  for (int i = 0; i < N; i++) {
    updateYaw(true);
    float a = yawDeg * DEG_TO_RAD;
    sx += cosf(a);
    sy += sinf(a);
    ok++;
    delay(10);
  }
  if (ok <= 0) return yawDeg;
  return atan2f(sy, sx) * RAD_TO_DEG;
}

void setup() {
  pinMode(LED_DEBUG_PIN, OUTPUT);
  digitalWrite(LED_DEBUG_PIN, LOW);
  blinkDebug(2, 120, 120);  // Dau hieu vao setup()

  Serial.begin(115200);
  delay(2000);  // cho Serial Monitor mo kip cong COM
  Serial.println(F("[TEST] Start test quay + drift"));
  Serial.println(F("[TEST] Build: with detailed diagnostics"));

  Serial.println(F("[BOOT] Wire.begin"));
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  // 100kHz: on dinh hon khi day Dupont / breadboard / nhieu thiet bi (giam chập chờn scan)
  Wire.setClock(100000);
  Serial.println(F("[BOOT] I2C clock=100kHz"));
  scanI2CBus();
  Serial.println(F("[BOOT] Motor pins"));
  setupMotorPins();
  stopMotors();

  Serial.println(F("[BOOT] setupIMU"));
  g_imuOk = setupIMU();
  Serial.println(F("[BOOT] setupMAG"));
  setupMagAuto();

  Serial.print(F("[TEST] IMU="));
  Serial.print(g_imuOk ? F("OK") : F("FAIL"));
  Serial.print(F(" MAG="));
  Serial.print(g_magOk ? F("OK") : F("FAIL"));
  Serial.print(F(" chip="));
  if (g_magChip == MAG_CHIP_AK09911) Serial.println(F("AK09911"));
  else if (g_magChip == MAG_CHIP_QMC5883) Serial.println(F("QMC5883"));
  else Serial.println(F("NONE"));

  if (!g_imuOk) {
    Serial.println(F("[TEST] IMU loi, dung test."));
    // Nhay nhanh lien tuc: IMU fail
    while (true) {
      digitalWrite(LED_DEBUG_PIN, HIGH);
      delay(80);
      digitalWrite(LED_DEBUG_PIN, LOW);
      delay(120);
    }
    return;
  }

  // Warmup filter
  Serial.println(F("[TEST] Warmup fusion..."));
  for (int i = 0; i < 40; i++) {
    updateYaw(true);
    delay(8);
  }

  // (1) Khi bat nguon lap tuc luu goc co ban
  baseHeadingDeg = captureBaseHeading();
  Serial.print(F("[TEST] Base heading = "));
  Serial.println(baseHeadingDeg, 2);
  blinkDebug(3, 80, 80);  // setup xong
}

void loop() {
  // Heartbeat LED: chop cham neu loop dang chay
  unsigned long nowBeat = millis();
  if (nowBeat - lastLedBeatMs >= 500) {
    lastLedBeatMs = nowBeat;
    digitalWrite(LED_DEBUG_PIN, !digitalRead(LED_DEBUG_PIN));
  }

  if (!g_imuOk) {
    stopMotors();
    delay(200);
    return;
  }

  updateYaw(true);
  float err = normalizeAngle180(baseHeadingDeg - yawDeg);
  float absErr = fabsf(err);

  // (2) Neu bi quay lech mot goc bat ky -> quay ve goc co ban (style final_2)
  if (absErr <= TOLERANCE_DEG) {
    stopMotors();
  } else {
    rotateToAngle(baseHeadingDeg);
  }

  unsigned long nowMs = millis();
  if (nowMs - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = nowMs;
    Serial.print(F("[HB] updates="));
    Serial.print(g_updateCount);
    Serial.print(F(" magFail="));
    Serial.print(g_magReadFailCount);
    Serial.print(F(" imu="));
    Serial.print(g_imuOk ? F("OK") : F("FAIL"));
    Serial.print(F(" mag="));
    Serial.print(g_magOk ? F("OK") : F("FAIL"));
    Serial.print(F(" chip="));
    if (g_magChip == MAG_CHIP_AK09911) Serial.print(F("AK"));
    else if (g_magChip == MAG_CHIP_QMC5883) Serial.print(F("QMC"));
    else Serial.print(F("-"));
    Serial.print(F(" | "));
    Serial.print(F("[DRIFT] yaw="));
    Serial.print(yawDeg, 2);
    Serial.print(F(" base="));
    Serial.print(baseHeadingDeg, 2);
    Serial.print(F(" err="));
    Serial.print(err, 2);
    Serial.print(F(" | hold="));
    Serial.println((absErr <= TOLERANCE_DEG) ? F("YES") : F("NO"));
  }

  delay(8);
}
