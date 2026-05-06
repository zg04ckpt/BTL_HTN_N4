/**
 * Test chẩn đoán MPU9250 + AK8963 (la bàn tích hợp).
 *
 * Mục tiêu:
 * 1) Quét I2C xem có địa chỉ 0x68 và 0x0C không.
 * 2) Đọc WHO_AM_I của MPU.
 * 3) Bật bypass và cấu hình AK8963.
 * 4) In ST1/ST2 + raw XYZ + yaw để xác định vì sao yaw đứng im.
 *
 * Lưu ý:
 * - File này là .cpp theo yêu cầu. Nếu Arduino IDE không nhận sketch,
 *   hãy đổi tên thành mpu9250_diag.ino trong cùng thư mục.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t MAG_ADDR = 0x0C;

constexpr uint8_t MPU_REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU_REG_INT_PIN_CFG = 0x37;

constexpr uint8_t AK_REG_WIA = 0x00;
constexpr uint8_t AK_REG_ST1 = 0x02;
constexpr uint8_t AK_REG_HXL = 0x03;
constexpr uint8_t AK_REG_CNTL1 = 0x0A;
constexpr uint8_t AK_MODE_CONT_100HZ_16BIT = 0x16;

static bool readBytes(uint8_t dev, uint8_t reg, uint8_t* out, uint8_t len) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom(dev, len);
  if (got < len) return false;
  for (uint8_t i = 0; i < len; i++) out[i] = Wire.read();
  return true;
}

static bool readByte(uint8_t dev, uint8_t reg, uint8_t& out) {
  return readBytes(dev, reg, &out, 1);
}

static void writeReg(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static void scanI2C() {
  Serial.println(F("=== I2C scan ==="));
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(F("Found: 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
    }
  }
}

static bool readMagRaw(int16_t& mx, int16_t& my, int16_t& mz, uint8_t& st1Out, uint8_t& st2Out) {
  st1Out = 0;
  st2Out = 0;

  if (!readByte(MAG_ADDR, AK_REG_ST1, st1Out)) return false;
  if ((st1Out & 0x01) == 0) return false;

  uint8_t buf[7] = {0};
  if (!readBytes(MAG_ADDR, AK_REG_HXL, buf, 7)) return false;

  mx = (int16_t)((buf[1] << 8) | buf[0]);
  my = (int16_t)((buf[3] << 8) | buf[2]);
  mz = (int16_t)((buf[5] << 8) | buf[4]);
  st2Out = buf[6];
  if (st2Out & 0x08) return false;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);  // để 100k cho ổn định lúc chẩn đoán
  delay(50);

  scanI2C();

  uint8_t who = 0;
  if (readByte(MPU_ADDR, MPU_REG_WHO_AM_I, who)) {
    Serial.print(F("MPU WHO_AM_I = 0x"));
    Serial.println(who, HEX);
  } else {
    Serial.println(F("Loi doc WHO_AM_I MPU"));
  }

  writeReg(MPU_ADDR, MPU_REG_PWR_MGMT_1, 0x00);
  delay(50);
  writeReg(MPU_ADDR, MPU_REG_INT_PIN_CFG, 0x02);
  delay(10);

  uint8_t wia = 0;
  if (readByte(MAG_ADDR, AK_REG_WIA, wia)) {
    Serial.print(F("AK8963 WIA = 0x"));
    Serial.println(wia, HEX);
  } else {
    Serial.println(F("Loi doc WIA AK8963"));
  }

  writeReg(MAG_ADDR, AK_REG_CNTL1, 0x00);
  delay(10);
  writeReg(MAG_ADDR, AK_REG_CNTL1, AK_MODE_CONT_100HZ_16BIT);
  delay(10);

  Serial.println(F("=== Bat dau doc MAG ==="));
  Serial.println(F("st1\tst2\trawX\trawY\trawZ\tyaw"));
}

void loop() {
  static unsigned long lastMs = 0;
  if (millis() - lastMs < 200) return;
  lastMs = millis();

  int16_t mx = 0, my = 0, mz = 0;
  uint8_t st1 = 0, st2 = 0;
  bool ok = readMagRaw(mx, my, mz, st1, st2);

  Serial.print(st1, HEX);
  Serial.print('\t');
  Serial.print(st2, HEX);
  Serial.print('\t');

  if (!ok) {
    Serial.println(F("MAG_FAIL"));
    return;
  }

  float yaw = atan2f((float)my, (float)mx) * 180.0f / PI;
  while (yaw > 180.0f) yaw -= 360.0f;
  while (yaw < -180.0f) yaw += 360.0f;

  Serial.print(mx);
  Serial.print('\t');
  Serial.print(my);
  Serial.print('\t');
  Serial.print(mz);
  Serial.print('\t');
  Serial.println(yaw, 2);
}

