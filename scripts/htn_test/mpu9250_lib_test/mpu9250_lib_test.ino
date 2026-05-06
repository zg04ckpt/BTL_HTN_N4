#include <Wire.h>
#include <MPU9250_asukiaaa.h>

// ESP32 pins
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

MPU9250_asukiaaa mySensor;

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println();
  Serial.println("=== MPU9250 LIB TEST (asukiaaa) ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  mySensor.setWire(&Wire);

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  Serial.println("Init done. Reading accel/gyro/mag...");
  Serial.println("ax\tay\taz\tgx\tgy\tgz\tmx\tmy\tmz\theading");
}

void loop() {
  int errA = mySensor.accelUpdate();
  int errG = mySensor.gyroUpdate();
  int errM = mySensor.magUpdate();

  Serial.print("errA=");
  Serial.print(errA);
  Serial.print(" errG=");
  Serial.print(errG);
  Serial.print(" errM=");
  Serial.println(errM);

  if (errA == 0 && errG == 0 && errM == 0) {
    float ax = mySensor.accelX();
    float ay = mySensor.accelY();
    float az = mySensor.accelZ();

    float gx = mySensor.gyroX();
    float gy = mySensor.gyroY();
    float gz = mySensor.gyroZ();

    float mx = mySensor.magX();
    float my = mySensor.magY();
    float mz = mySensor.magZ();

    float heading = atan2f(my, mx) * 180.0f / PI;
    if (heading < 0) heading += 360.0f;

    Serial.print(ax, 3); Serial.print('\t');
    Serial.print(ay, 3); Serial.print('\t');
    Serial.print(az, 3); Serial.print('\t');
    Serial.print(gx, 3); Serial.print('\t');
    Serial.print(gy, 3); Serial.print('\t');
    Serial.print(gz, 3); Serial.print('\t');
    Serial.print(mx, 3); Serial.print('\t');
    Serial.print(my, 3); Serial.print('\t');
    Serial.print(mz, 3); Serial.print('\t');
    Serial.println(heading, 2);
  } else {
    Serial.println("Doc cam bien loi. Neu errM != 0 lien tuc thi mag khong hoat dong.");
  }

  delay(500);
}

