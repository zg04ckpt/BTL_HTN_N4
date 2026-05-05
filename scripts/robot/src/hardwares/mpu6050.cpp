#include "mpu6050.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

namespace {

constexpr int MPU_SDA_PIN = 21;
constexpr int MPU_SCL_PIN = 22;
constexpr uint8_t MPU_ADDR = 0x68;

// Thanh ghi MPU6050
constexpr uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU_REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU_REG_GYRO_ZOUT_H = 0x47;

// Gyro ±250 dps => 131 LSB/(deg/s)
constexpr float GYRO_SCALE_DPS = 131.0f;

// Chặn nhiễu nhỏ quanh 0
constexpr float GYRO_DEADBAND_DPS = 0.10f;
// Lọc thông thấp cho vận tốc góc Z
constexpr float GYRO_LPF_ALPHA = 0.22f;
// Ngưỡng coi là đứng yên (deg/s) để cập nhật lại bias
constexpr float STATIONARY_DPS = 0.35f;
// Số mẫu liên tiếp đứng yên mới cho phép cập nhật bias
constexpr int STATIONARY_COUNT_REQUIRED = 25;
// Tốc độ cập nhật bias khi đứng yên (0..1)
constexpr float BIAS_ADAPT_ALPHA = 0.02f;

float gyroOffsetZ = 0.0f;
unsigned long lastGyroMs = 0;
float filteredRateZ = 0.0f;
int stationaryCount = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

bool readReg(uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) < 1) return false;
  out = Wire.read();
  return true;
}

bool readGyroZRaw(int16_t& rawZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_GYRO_ZOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) < 2) return false;
  rawZ = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

}  // namespace

float yawDeg = 0.0f;

void setupMPU6050() {
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000);

  uint8_t who = 0;
  if (readReg(MPU_REG_WHO_AM_I, who)) {
    Serial.print(F("[MPU6050] WHO_AM_I = 0x"));
    Serial.println(who, HEX);
  } else {
    Serial.println(F("[MPU6050] Loi doc WHO_AM_I"));
  }

  // Đánh thức cảm biến
  writeReg(MPU_REG_PWR_MGMT_1, 0x00);
  delay(50);

  // Hiệu chuẩn offset gyro Z khi để yên
  long sum = 0;
  int samples = 0;
  for (int i = 0; i < 600; i++) {
    int16_t raw = 0;
    if (readGyroZRaw(raw)) {
      sum += raw;
      samples++;
    }
    delay(2);
  }
  gyroOffsetZ = (samples > 0) ? ((float)sum / (float)samples) : 0.0f;

  yawDeg = 0.0f;
  lastGyroMs = millis();
  filteredRateZ = 0.0f;
  stationaryCount = 0;

  Serial.print(F("[MPU6050] GyroZ offset = "));
  Serial.println(gyroOffsetZ, 2);
}

float readYawDeg() { return yawDeg; }

void updateYawDeg() {
  int16_t rawZ = 0;
  if (!readGyroZRaw(rawZ)) return;

  const unsigned long now = millis();
  float dt = (now - lastGyroMs) / 1000.0f;
  lastGyroMs = now;
  if (dt <= 0.0f || dt > 0.2f) return;

  // Tốc độ góc thô sau khi trừ offset
  float rateDps = ((float)rawZ - gyroOffsetZ) / GYRO_SCALE_DPS;
  // Lọc thông thấp để giảm nhiễu cao tần
  filteredRateZ = GYRO_LPF_ALPHA * rateDps + (1.0f - GYRO_LPF_ALPHA) * filteredRateZ;

  // Nếu cảm biến đứng yên đủ lâu thì cập nhật lại bias từ raw hiện tại
  // để giảm trôi tích lũy khi robot dừng.
  if (fabs(filteredRateZ) < STATIONARY_DPS) {
    stationaryCount++;
    if (stationaryCount >= STATIONARY_COUNT_REQUIRED) {
      gyroOffsetZ = (1.0f - BIAS_ADAPT_ALPHA) * gyroOffsetZ + BIAS_ADAPT_ALPHA * (float)rawZ;
      stationaryCount = STATIONARY_COUNT_REQUIRED;
    }
  } else {
    stationaryCount = 0;
  }

  // Vùng chết quanh 0 sau lọc
  if (fabs(filteredRateZ) < GYRO_DEADBAND_DPS) return;

  yawDeg += filteredRateZ * dt;
}

