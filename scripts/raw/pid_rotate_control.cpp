/*
 * PID rotate control (ESP32 + MPU6050 + 2 encoder channels)
 * 1) Góc quay tức thời lấy từ MPU6050 (gyro Z tích phân theo thời gian)
 * 2) PID cân bằng lực quay 2 bánh bằng sai lệch xung encoder
 * 3) Có 2 chế độ quay: quay 1 bánh và quay 2 bánh
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <math.h>
 
 const int ENA = 13, IN1 = 12, IN2 = 14;  // Left motor
 const int ENB = 25, IN3 = 27, IN4 = 26;  // Right motor
 const int SDA_PIN = 21;
 const int SCL_PIN = 22;
 const uint8_t MPU_ADDR = 0x68;
 
 const int ENCODER_LEFT_PIN = 34;
 const int ENCODER_RIGHT_PIN = 35;
 
// PWM 8-bit thông thường: analogWrite 0..255
 
 volatile long encoderLeftCount = 0;
 volatile long encoderRightCount = 0;
 
 float yawDeg = 0.0f;            // Góc quay tức thời (độ)
 float gyroZOffset = 0.0f;       // Offset gyro Z (deg/s)
 unsigned long lastImuMicros = 0;
 
 // PID cân bằng 2 bánh theo chênh lệch encoder
 float kpBal = 2.0f;
 float kiBal = 0.01f;
 float kdBal = 0.2f;
 float balIntegral = 0.0f;
 float balPrevErr = 0.0f;
 unsigned long balPrevMs = 0;
 
 // ---------- Encoder ISR ----------
 void IRAM_ATTR onLeftEncoderPulse() { encoderLeftCount++; }
 void IRAM_ATTR onRightEncoderPulse() { encoderRightCount++; }
 
 // ---------- Motor helpers ----------
 int clampPwm(int v) {
   if (v < 0) return 0;
   if (v > 255) return 255;
   return v;
 }
 
 void setLeftMotor(bool forward, int pwm) {
   pwm = clampPwm(pwm);
   digitalWrite(IN1, forward ? HIGH : LOW);
   digitalWrite(IN2, forward ? LOW : HIGH);
  analogWrite(ENA, pwm);
 }
 
 void setRightMotor(bool forward, int pwm) {
   pwm = clampPwm(pwm);
   digitalWrite(IN3, forward ? HIGH : LOW);
   digitalWrite(IN4, forward ? LOW : HIGH);
  analogWrite(ENB, pwm);
 }
 
 void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
   digitalWrite(IN1, LOW);
   digitalWrite(IN2, LOW);
   digitalWrite(IN3, LOW);
   digitalWrite(IN4, LOW);
 }
 
 // ---------- MPU6050 helpers ----------
 void writeMpuReg(uint8_t reg, uint8_t value) {
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(reg);
   Wire.write(value);
   Wire.endTransmission();
 }
 
 int16_t readMpuInt16(uint8_t reg) {
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(reg);
   Wire.endTransmission(false);
   Wire.requestFrom((int)MPU_ADDR, 2);
 
   int16_t hi = Wire.read();
   int16_t lo = Wire.read();
   return (int16_t)((hi << 8) | lo);
 }
 
 void initMpu6050() {
   writeMpuReg(0x6B, 0x00);  // Wake up MPU6050
   writeMpuReg(0x1B, 0x00);  // Gyro range +/-250 deg/s
   delay(100);
 }
 
 void calibrateGyroZOffset() {
   const int samples = 1000;
   long sum = 0;
   for (int i = 0; i < samples; i++) {
     sum += readMpuInt16(0x47);  // GYRO_ZOUT_H
     delay(2);
   }
   // 131 LSB/(deg/s) ở full-scale +/-250deg/s
   gyroZOffset = (sum / (float)samples) / 131.0f;
 }
 
 void updateYawFromMpu() {
   unsigned long nowUs = micros();
   if (lastImuMicros == 0) {
     lastImuMicros = nowUs;
     return;
   }
 
   float dt = (nowUs - lastImuMicros) / 1000000.0f;
   lastImuMicros = nowUs;
   if (dt <= 0 || dt > 0.1f) return;  // tránh spike thời gian
 
   float gyroZDegPerSec = readMpuInt16(0x47) / 131.0f - gyroZOffset;
   yawDeg += gyroZDegPerSec * dt;
 }
 
 void resetYawAndEncoderForRotate() {
   noInterrupts();
   encoderLeftCount = 0;
   encoderRightCount = 0;
   interrupts();
 
   yawDeg = 0.0f;
   balIntegral = 0.0f;
   balPrevErr = 0.0f;
   balPrevMs = millis();
   lastImuMicros = micros();
 }
 
 // ---------- PID balance ----------
 float computeBalancePid() {
   unsigned long nowMs = millis();
   float dt = (nowMs - balPrevMs) / 1000.0f;
   if (dt <= 0.0f) dt = 0.001f;
   balPrevMs = nowMs;
 
   long leftC, rightC;
   noInterrupts();
   leftC = encoderLeftCount;
   rightC = encoderRightCount;
   interrupts();
 
   float err = (float)(leftC - rightC);  // lệch xung
   balIntegral += err * dt;
   float derivative = (err - balPrevErr) / dt;
   balPrevErr = err;
 
   return kpBal * err + kiBal * balIntegral + kdBal * derivative;
 }
 
 // ---------- Rotate modes ----------
 // Quay 2 bánh: 2 bánh quay ngược chiều để robot quay tại chỗ
 void rotateTwoWheelMode(float targetAngleDeg, bool clockwise, int basePwm) {
   resetYawAndEncoderForRotate();
 
   while (fabs(yawDeg) < targetAngleDeg) {
     updateYawFromMpu();
     float correction = computeBalancePid();
 
     int leftPwm = basePwm;
     int rightPwm = basePwm;
 
     // Bù PWM để giảm chênh lệch xung encoder
     leftPwm -= (int)correction;
     rightPwm += (int)correction;
 
     if (clockwise) {
       setLeftMotor(true, leftPwm);
       setRightMotor(false, rightPwm);
     } else {
       setLeftMotor(false, leftPwm);
       setRightMotor(true, rightPwm);
     }
     delay(2);
   }
   stopMotors();
 }
 
 // Quay 1 bánh: khóa 1 bánh, 1 bánh còn lại quay để đổi hướng
 void rotateOneWheelMode(float targetAngleDeg, bool clockwise, int basePwm) {
   resetYawAndEncoderForRotate();
 
   while (fabs(yawDeg) < targetAngleDeg) {
     updateYawFromMpu();
 
     if (clockwise) {
       // Giữ bánh trái đứng yên, quay bánh phải lùi để robot quay phải
       setLeftMotor(true, 0);
       setRightMotor(false, basePwm);
     } else {
       // Giữ bánh phải đứng yên, quay bánh trái lùi để robot quay trái
       setRightMotor(true, 0);
       setLeftMotor(false, basePwm);
     }
     delay(2);
   }
   stopMotors();
 }
 
 void setup() {
   Serial.begin(115200);
   Wire.begin(SDA_PIN, SCL_PIN);
 
   pinMode(IN1, OUTPUT);
   pinMode(IN2, OUTPUT);
   pinMode(IN3, OUTPUT);
   pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
 
  // GPIO34/35 là input-only, không hỗ trợ pull-up nội trên ESP32.
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);
   attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), onLeftEncoderPulse, RISING);
   attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), onRightEncoderPulse, RISING);
 
   initMpu6050();
   calibrateGyroZOffset();
 
   Serial.println("Rotate PID control ready.");
 }
 
 void loop() {
   // Demo: quay phải 90 do bang 2 banh, dung 1s, quay trai 45 do bang 1 banh.
   rotateTwoWheelMode(90.0f, true, 120);
   delay(1000);
   // rotateOneWheelMode(45.0f, false, 120);
   // delay(2000);
 }
 