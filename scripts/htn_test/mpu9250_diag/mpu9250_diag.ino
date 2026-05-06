#include <Arduino.h>
#include <Wire.h>

static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

static const uint8_t MPU_ADDR = 0x68;
static const uint8_t MAG_ADDR = 0x0C;

// MPU registers
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_INT_PIN_CFG = 0x37;
static const uint8_t REG_USER_CTRL = 0x6A;
static const uint8_t REG_WHO_AM_I = 0x75;

// AK8963 registers
static const uint8_t REG_AK_WIA = 0x00;
static const uint8_t REG_AK_CNTL1 = 0x0A;
static const uint8_t AK_MODE_CONT_100HZ_16BIT = 0x16;

static bool i2cReadByte(uint8_t dev, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(dev, (uint8_t)1) < 1) return false;
  val = Wire.read();
  return true;
}

static void i2cWriteByte(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static void scanI2C(const char* title) {
  Serial.println(title);
  int n = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C device found at 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      n++;
    }
  }
  if (n == 0) Serial.println("No I2C devices found");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1500); // Cho Serial Monitor bat kip
  Serial.println();
  Serial.println("=== BOOT MPU9250 DIAG ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(50);

  scanI2C("=== Scan truoc bypass ===");

  // Danh thuc MPU
  i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x00);
  delay(50);

  // Tat I2C master noi bo cua MPU
  i2cWriteByte(MPU_ADDR, REG_USER_CTRL, 0x00);
  delay(10);

  // Bat bypass de ESP32 thay AK8963 tai 0x0C
  i2cWriteByte(MPU_ADDR, REG_INT_PIN_CFG, 0x02);
  delay(10);

  scanI2C("=== Scan sau bypass ===");

  uint8_t who = 0;
  if (i2cReadByte(MPU_ADDR, REG_WHO_AM_I, who)) {
    Serial.print("MPU WHO_AM_I = 0x");
    Serial.println(who, HEX);
  } else {
    Serial.println("Doc WHO_AM_I MPU that bai");
  }

  uint8_t wia = 0;
  if (i2cReadByte(MAG_ADDR, REG_AK_WIA, wia)) {
    Serial.print("AK8963 WIA = 0x");
    Serial.println(wia, HEX);
  } else {
    Serial.println("Doc WIA AK8963 that bai");
  }

  i2cWriteByte(MAG_ADDR, REG_AK_CNTL1, 0x00);
  delay(10);
  i2cWriteByte(MAG_ADDR, REG_AK_CNTL1, AK_MODE_CONT_100HZ_16BIT);
  delay(10);

  Serial.println("=== DONE SETUP ===");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last >= 2000) {
    last = millis();
    Serial.println("[alive] diag dang chay...");
  }
}

