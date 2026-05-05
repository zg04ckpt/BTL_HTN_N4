#include <Wire.h>

// ================= CẤU HÌNH CHÂN =================
const int ENA = 13, IN1 = 12, IN2 = 14; 
const int ENB = 25, IN3 = 27, IN4 = 26; 
const int TRIG_R = 32, ECHO_R = 33;
const int TRIG_M = 4,  ECHO_M = 5;
const int TRIG_L = 18, ECHO_L = 19;
const int ENCODER_LEFT = 23;
const int MPU_ADDR = 0x68;

// ================= BIẾN TOÀN CỤC =================
volatile long pulseCount = 0;
float gyroZ = 0, gyroOffsetZ = 0;
float currentYaw = 0, targetYaw = 0;
unsigned long lastTime = 0;
unsigned long lastSerialTime = 0; // Để khống chế tốc độ in Serial

int state = 0; 
int turnDirection = 1; 
int baseSpeed = 80;    // Đã chỉnh lại mức 80 cho mượt, 30 thường bị yếu motor

float Kp = 3.5; 
float distL, distM, distR; // Lưu khoảng cách 3 cảm biến

// ================= HÀM NGẮT =================
void IRAM_ATTR countPulse() {
  pulseCount++;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  
  // CHỜ SERIAL KẾT NỐI (tối đa 3 giây)
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000) {
    delay(10);
  }
  delay(500);
  Serial.println("\n\n--- KHOI DONG ROBOT ---");

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_M, OUTPUT); pinMode(ECHO_M, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  attachInterrupt(ENCODER_LEFT, countPulse, FALLING);

  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); 
  Wire.endTransmission(true);

  Serial.println("--- ĐANG HIỆU CHỈNH MPU6050 (GIỮ YÊN ROBOT) ---");
  long sumZ = 0;
  for(int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    sumZ += (Wire.read() << 8 | Wire.read());
    delay(3);
  }
  gyroOffsetZ = sumZ / 500.0;
  Serial.print("Offset Z = ");
  Serial.println(gyroOffsetZ);
  lastTime = millis();
  
  Serial.println("--- ROBOT SAN SANG! ---");
  delay(1000);
}

// ================= HÀM TIỆN ÍCH =================
float getDist(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10); digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 15000); // Timeout 15ms cho nhanh
  if(dur == 0) return 999.0;
  return (dur * 0.0343) / 2;
}

void updateYaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t rawZ = Wire.read() << 8 | Wire.read();
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  gyroZ = (rawZ - gyroOffsetZ) / 131.0; 
  if (abs(gyroZ) > 1.0) { 
    currentYaw += gyroZ * dt; 
  }
}

void setMotors(int speedL, int speedR) {
  if (speedL >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); speedL = -speedL; }
  analogWrite(ENA, constrain(speedL, 0, 255));

  if (speedR >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); speedR = -speedR; }
  analogWrite(ENB, constrain(speedR, 0, 255));
}

void stopMotors() {
  // Phanh gấp chủ động
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255); analogWrite(ENB, 255);
  delay(150); 
  // Ngắt điện hẳn
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ================= VÒNG LẶP CHÍNH =================
void loop() {
  updateYaw(); 

  // Đọc khoảng cách 3 phía
  distL = getDist(TRIG_L, ECHO_L);
  delay(20);
  distM = getDist(TRIG_M, ECHO_M);
  delay(20);
  distR = getDist(TRIG_R, ECHO_R);

  // --- IN DỮ LIỆU RA SERIAL (MỖI 300MS) ---
  if (millis() - lastSerialTime > 300) {
    Serial.print("Sieu am(L-M-R): ");
    Serial.print(distL, 1); Serial.print(" - ");
    Serial.print(distM, 1); Serial.print(" - ");
    Serial.print(distR, 1);
    Serial.print(" | Xung: "); Serial.print(pulseCount);
    Serial.print(" | gyroZ: "); Serial.print(gyroZ, 1);
    Serial.print(" | Goc: "); Serial.print(currentYaw, 1);
    Serial.print(" | State: "); Serial.println(state);
    lastSerialTime = millis();
  }

  // --- MÁY TRẠNG THÁI (STATE MACHINE) ---
  switch (state) {
    case 0: { // ĐI THẲNG
      if (distM < 30.0 || distL < 15.0 || distR < 15.0) { 
        stopMotors();
        targetYaw = currentYaw + (90.0 * turnDirection);
        state = 1;
        delay(500);
        break;
      }
      float error = targetYaw - currentYaw;
      int correction = Kp * error;
      setMotors(baseSpeed - correction, baseSpeed + correction);
      break;
    }

    case 1: { // XOAY LẦN 1
      float error = targetYaw - currentYaw;
      if (abs(error) < 3.0) {
        stopMotors();
        pulseCount = 0; 
        targetYaw = currentYaw;
        state = 2;
        delay(500);
      } else {
        int turnSpeed = 90;
        if (error > 0) setMotors(turnSpeed, -turnSpeed);
        else setMotors(-turnSpeed, turnSpeed);
      }
      break;
    }

    case 2: { // CHUYỂN LÀN (NHÍCH LÊN)
      if (pulseCount >= 20 || distM < 15.0) { 
        stopMotors();
        targetYaw = currentYaw + (90.0 * turnDirection);
        state = 3;
        delay(500);
        break;
      }
      float error = targetYaw - currentYaw;
      setMotors(baseSpeed - (Kp * error), baseSpeed + (Kp * error));
      break;
    }

    case 3: { // XOAY LẦN 2 (QUAY ĐẦU)
      float error = targetYaw - currentYaw;
      if (abs(error) < 3.0) {
        stopMotors();
        targetYaw = currentYaw; 
        turnDirection = -turnDirection; // Đổi hướng quẹo cho lần sau
        state = 0;
        delay(500);
      } else {
        int turnSpeed = 90;
        if (error > 0) setMotors(turnSpeed, -turnSpeed);
        else setMotors(-turnSpeed, turnSpeed);
      }
      break;
    }
  }
}