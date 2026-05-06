/**
 * Test AHRS: Madgwick 6DOF / 9DOF, so sánh với tích phân gyro thuần.
 *
 * === Thực tế về "độ chính xác tuyệt đối" ===
 * - IMU tích phân (gyro) không bao giờ cho hướng tuyệt đối dài hạn: luôn có trôi.
 * - 6DOF (accel+gyro): bù roll/pitch tương đối ổn; yaw vẫn trôi (không quan sát hướng tuyệt đối).
 * - 9DOF + từ kế (AK8963 trên MPU9250): yaw bám "Bắc từ" trong điều kiện từ trường ổn định —
 *   vẫn không phải Bắc địa lý (cần độ lệch declination), và trong nhà sắt / động cơ làm sai lệch mag.
 * - Để định hướng "chuẩn thực sự": GPS đầu xe / encoder + bản đồ / camera marker / la bàn tham chiếu ngoài.
 *
 * Sketch này đẩy mọi thứ có thể trên MCU: fusion Madgwick, hiệu chuẩn gyro, hiệu chuẩn mag (min–max + lưu),
 * deadband gyro, tùy chọn khai từ, clock I2C tùy chỉnh.
 *
 * Thư viện: MPU6050 (Electronic Cats), MadgwickAHRS, Preferences (ESP32, built-in).
 * Chân: SDA=21, SCL=22. MPU9250: bật I2C bypass để đọc AK8963 tại 0x0C.
 *
 * Arduino IDE: tên thư mục sketch = tên file .ino.
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include <Preferences.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const int PIN_SDA = 21;
static const int PIN_SCL = 22;

/** Thử 400 kHz; nếu dây dài / nhiễu, hạ xuống 100000. */
static const uint32_t I2C_CLOCK_HZ = 400000;

/** Bỏ qua tốc độ góc rất nhỏ (sau khi hiệu chuẩn) — giảm tích lũy nhiễu khi đứng yên. */
static const float GYRO_DEADBAND_DPS = 0.04f;

/**
 * Khai từ từ (°): Bắc từ - Bắc địa lý tại vị trí bạn (tra bản đồ NOAA/WMM).
 * Việt Nam khoảng vài độ; chỉnh để Serial hiển thị yaw gần la bàn địa lý hơn.
 */
static const float MAG_DECLINATION_DEG = 0.0f;

static const uint8_t MPU_ADDR = 0x68;
static const uint8_t MAG_ADDR = 0x0C;
static const uint8_t AK8963_REG_ST1 = 0x02;
static const uint8_t AK8963_REG_HXL = 0x03;
static const uint8_t AK8963_REG_ST2 = 0x09;
static const uint8_t AK8963_REG_CNTL1 = 0x0A;
static const uint8_t AK8963_MODE_CONT_100HZ_16B = 0x16;
static const uint8_t MPU_REG_INT_PIN_CFG = 0x37;
static const uint8_t MPU_REG_WHO_AM_I = 0x75;

static MPU6050 mpu(MPU6050_DEFAULT_ADDRESS, &Wire);
static Madgwick fusion;
static Preferences prefs;

static unsigned long prevFusionMicros = 0;
static unsigned long lastPrintMs = 0;

static float yawChiGyroDeg = 0.0f;
static uint8_t chipWhoAmI = 0;
static bool s_isMpu9250 = false;
static bool s_magInitialized = false;

/** Hiệu chuẫn mag đơn giản: offset + scale (min–max / ellipsoid trụ). Mag đầy đủ cần MotionCal / ma trận soft-iron. */
static float magOx = 0, magOy = 0, magOz = 0;
static float magSx = 1, magSy = 1, magSz = 1;
static bool magCalValid = false;

static void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static uint8_t mpuReadReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

static void magWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static bool magReadRaw(int16_t* mx, int16_t* my, int16_t* mz) {
  if (!mx || !my || !mz) return false;
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(AK8963_REG_ST1);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)1);
  if (Wire.available() < 1) return false;
  uint8_t st1 = Wire.read();
  if (!(st1 & 0x01)) return false;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(AK8963_REG_HXL);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)7);
  if (Wire.available() < 7) return false;
  uint8_t xl = Wire.read();
  uint8_t xh = Wire.read();
  uint8_t yl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t zl = Wire.read();
  uint8_t zh = Wire.read();
  uint8_t st2 = Wire.read();
  if (st2 & 0x08) return false;
  *mx = (int16_t)(((int16_t)xh << 8) | xl);
  *my = (int16_t)(((int16_t)yh << 8) | yl);
  *mz = (int16_t)(((int16_t)zh << 8) | zl);
  return true;
}

static bool initAk8963() {
  magWriteReg(AK8963_REG_CNTL1, 0x00);
  delay(10);
  magWriteReg(AK8963_REG_CNTL1, AK8963_MODE_CONT_100HZ_16B);
  delay(10);
  int16_t mx, my, mz;
  for (int i = 0; i < 30; i++) {
    if (magReadRaw(&mx, &my, &mz)) return true;
    delay(3);
  }
  return false;
}

/** Cho phép I2C ngoài truy cập AK8963 (MPU9250). */
static bool enableMpuI2cBypass() {
  mpuWriteReg(MPU_REG_INT_PIN_CFG, 0x02);
  delay(2);
  return true;
}

static void applyGyroDeadband(float* gxd, float* gyd, float* gzd) {
  if (fabsf(*gxd) < GYRO_DEADBAND_DPS) *gxd = 0.0f;
  if (fabsf(*gyd) < GYRO_DEADBAND_DPS) *gyd = 0.0f;
  if (fabsf(*gzd) < GYRO_DEADBAND_DPS) *gzd = 0.0f;
}

static void loadMagCalFromNvs() {
  prefs.begin("test_qu_mag", true);
  magCalValid = prefs.getBool("valid", false);
  if (magCalValid) {
    magOx = prefs.getFloat("ox", 0);
    magOy = prefs.getFloat("oy", 0);
    magOz = prefs.getFloat("oz", 0);
    magSx = prefs.getFloat("sx", 1);
    magSy = prefs.getFloat("sy", 1);
    magSz = prefs.getFloat("sz", 1);
    Serial.println(F("[Mag] Da nap hieu chuan tu NVS."));
  } else {
    Serial.println(F("[Mag] Chua co hieu chuan NVS — se chay figure-8."));
  }
  prefs.end();
}

static void saveMagCalToNvs() {
  prefs.begin("test_qu_mag", false);
  prefs.putBool("valid", true);
  prefs.putFloat("ox", magOx);
  prefs.putFloat("oy", magOy);
  prefs.putFloat("oz", magOz);
  prefs.putFloat("sx", magSx);
  prefs.putFloat("sy", magSy);
  prefs.putFloat("sz", magSz);
  prefs.end();
  magCalValid = true;
  Serial.println(F("[Mag] Da luu hieu chuan vao NVS."));
}

/**
 * Hiệu chuẩn "xoay đủ hướng": min–max mỗi trục → offset + scale (ứng xử hard iron đơn giản).
 * Soft iron đầy đủ = ma trận 3x3 (MotionCal + Visualizer).
 */
static void calibrateMagFigureEight() {
  Serial.println(F("\n=== XOAY CAM BIEN MAG ~20 giay (figure-8, phu het huong) ==="));
  int16_t mnX = 32767, mnY = 32767, mnZ = 32767;
  int16_t mxX = -32768, mxY = -32768, mxZ = -32768;
  unsigned long t0 = millis();
  while (millis() - t0 < 20000) {
    int16_t mx, my, mz;
    if (magReadRaw(&mx, &my, &mz)) {
      if (mx < mnX) mnX = mx;
      if (mx > mxX) mxX = mx;
      if (my < mnY) mnY = my;
      if (my > mxY) mxY = my;
      if (mz < mnZ) mnZ = mz;
      if (mz > mxZ) mxZ = mz;
    }
    delay(5);
  }
  float rx = (float)(mxX - mnX);
  float ry = (float)(mxY - mnY);
  float rz = (float)(mxZ - mnZ);
  if (rx < 80.0f || ry < 80.0f || rz < 80.0f) {
    Serial.println(F("[Mag] Pham vi qua nho — hieu chuan that bai, dung gia tri mac dinh."));
    magOx = magOy = magOz = 0;
    magSx = magSy = magSz = 1;
    return;
  }
  magOx = 0.5f * (float)(mxX + mnX);
  magOy = 0.5f * (float)(mxY + mnY);
  magOz = 0.5f * (float)(mxZ + mnZ);
  magSx = 2.0f / rx;
  magSy = 2.0f / ry;
  magSz = 2.0f / rz;
  saveMagCalToNvs();
}

static void inTenChip(uint8_t who) {
  Serial.print(F("WHO_AM_I = 0x"));
  Serial.print(who, HEX);
  Serial.print(F(" => "));
  switch (who) {
    case 0x68:
      Serial.println(F("MPU6000/6050"));
      break;
    case 0x71:
      Serial.println(F("MPU9250 (+ AK8963)"));
      break;
    case 0x73:
      Serial.println(F("MPU9255"));
      break;
    default:
      Serial.println(F("Khac / kiem tra phan cung"));
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  chipWhoAmI = mpuReadReg(MPU_REG_WHO_AM_I);
  inTenChip(chipWhoAmI);
  s_isMpu9250 = (chipWhoAmI == 0x71 || chipWhoAmI == 0x73);

  mpu.initialize(ACCEL_FS::A2G, GYRO_FS::G250DPS);
  if (!mpu.testConnection()) {
    Serial.println(F("[Canh bao] testConnection fail — van tiep tuc (MPU9250 co ID 0x71)."));
  }

  Serial.println(F("CalibrateGyro — giu IMU nam yen..."));
  mpu.CalibrateGyro(6);

  if (s_isMpu9250) {
    enableMpuI2cBypass();
    s_magInitialized = initAk8963();
    if (s_magInitialized) {
      loadMagCalFromNvs();
      if (!magCalValid) {
        calibrateMagFigureEight();
      }
    } else {
      Serial.println(F("[Mag] Khoi tao AK8963 that bai — dung 6DOF."));
    }
  }

  Serial.println(F("\n--- Chay: fusion + goc chi gyro ---"));
  Serial.println(F("Gui R trong Serial de hieu chuan mag lai (MPU9250)."));
  Serial.println(
      F("ms\tmode\tyaw\t yaw+decl\troll\tpitch\tgyroZ\t| yaw chi gyro (tich phan gyro)"));

  prevFusionMicros = micros();
  lastPrintMs = millis();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) {
      (void)Serial.read();
    }
    if ((c == 'r' || c == 'R') && s_isMpu9250 && s_magInitialized) {
      Serial.println(F("[Mag] Hieu chuan lai (figure-8)..."));
      calibrateMagFigureEight();
    }
  }

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
  float axg = ax * ar;
  float ayg = ay * ar;
  float azg = az * ar;
  float gxd = gx * gr;
  float gyd = gy * gr;
  float gzd = gz * gr;

  applyGyroDeadband(&gxd, &gyd, &gzd);

  bool used9Dof = false;
  if (s_isMpu9250 && s_magInitialized && magCalValid) {
    int16_t mrx, mry, mrz;
    if (magReadRaw(&mrx, &mry, &mrz)) {
      float mx = ((float)mrx - magOx) * magSx;
      float my = ((float)mry - magOy) * magSy;
      float mz = ((float)mrz - magOz) * magSz;
      fusion.update(gxd, gyd, gzd, axg, ayg, azg, mx, my, mz);
      used9Dof = true;
    } else {
      fusion.updateIMU(gxd, gyd, gzd, axg, ayg, azg);
    }
  } else {
    fusion.updateIMU(gxd, gyd, gzd, axg, ayg, azg);
  }

  float yawMadg = fusion.getYaw();
  float rollDeg = fusion.getRoll();
  float pitchDeg = fusion.getPitch();
  float yawPlusDecl = yawMadg + MAG_DECLINATION_DEG;
  while (yawPlusDecl > 180.0f) yawPlusDecl -= 360.0f;
  while (yawPlusDecl < -180.0f) yawPlusDecl += 360.0f;

  yawChiGyroDeg += gzd * dt;

  unsigned long m = millis();
  if (m - lastPrintMs >= 500) {
    lastPrintMs = m;
    Serial.print(m);
    Serial.print('\t');
    Serial.print(used9Dof ? F("9D") : F("6D"));
    Serial.print('\t');
    Serial.print(yawMadg, 3);
    Serial.print('\t');
    Serial.print(yawPlusDecl, 3);
    Serial.print('\t');
    Serial.print(rollDeg, 2);
    Serial.print('\t');
    Serial.print(pitchDeg, 2);
    Serial.print('\t');
    Serial.print(gzd, 4);
    Serial.print('\t');
    Serial.println(yawChiGyroDeg, 3);
  }
}
