#include <Arduino.h>
#include <Wire.h>

/**
 * FILE: pid_rotate_control.cpp
 * MÔ TẢ: Sketch hoàn chỉnh để test quay góc PID (Đã sửa lỗi khai báo hàm).
 */

// ================= CẤU HÌNH PIN & PHẦN CỨNG =================
const int ENA = 13, IN1 = 12, IN2 = 14;
const int ENB = 25, IN3 = 27, IN4 = 26;
const uint8_t MPU_ADDR = 0x68;
const float GYRO_SCALE = 131.0f;

// --- Tỉ lệ lực động cơ (Cân bằng nếu motor không đều) ---
const float LEFT_GAIN  = 1.0f;
const float RIGHT_GAIN = 1.0f; 

// ================= BIẾN HỆ THỐNG =================
float yawDeg = 0, gyroOffsetZ = 0;
unsigned long lastGyroMs = 0;

// ================= THÔNG SỐ PID QUAY GÓC (SIÊU MƯỢT) =================
float Kp_rot = 1.0f;           
float Kd_rot = 0.5f;           
float rotate_tolerance = 2.0f; // Chấp nhận sai số 2 độ để robot dừng êm
int min_turn_speed = 30;       // Tốc độ nhích cực thấp
int max_turn_speed = 80;       
const int max_correction_swings = 3;

float normalizeAngle180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// ================= CÁC HÀM CƠ BẢN =================

void setMotors(int speedL, int speedR) {
    int leftCmd = (int)(speedL * LEFT_GAIN);
    int rightCmd = -(int)(speedR * RIGHT_GAIN); // Đảo chiều bánh phải

    // Bánh Trái
    if (leftCmd >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
    else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); leftCmd = -leftCmd; }
    analogWrite(ENA, constrain(leftCmd, 0, 150));

    // Bánh Phải
    if (rightCmd >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
    else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); rightCmd = -rightCmd; }
    analogWrite(ENB, constrain(rightCmd, 0, 150));
}

void updateYaw() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return;
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    
    unsigned long now = millis();
    float dt = (now - lastGyroMs) / 1000.0f;
    lastGyroMs = now;
    if (dt <= 0 || dt > 0.1f) return;
    
    yawDeg += ((rawZ - gyroOffsetZ) / GYRO_SCALE) * dt;
}

// ================= HÀM QUAY GÓC PID CHÍNH =================

void rotateToAnglePID(float targetAngle) {
    float lastError = 0;
    unsigned long lastTime = millis();
    unsigned long startTime = millis();
    int correctionSwings = 0;
    bool hasPrevError = false;
    float prevError = 0.0f;
    
    Serial.print("[ROTATE] Dang quay den: "); Serial.println(targetAngle);

    while (true) {
        updateYaw();
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0) continue;
        lastTime = now;

        float error = normalizeAngle180(targetAngle - yawDeg);
        if (fabsf(error) < rotate_tolerance) {
            setMotors(0, 0);
            delay(100); // Chờ ổn định
            Serial.print("[DONE] Yaw hien tai: "); Serial.println(yawDeg);
            break;
        }

        if (hasPrevError && (error * prevError < 0.0f)) {
            correctionSwings++;
            Serial.print("[INFO] correction swings: ");
            Serial.println(correctionSwings);
        }
        prevError = error;
        hasPrevError = true;

        if (correctionSwings >= max_correction_swings) {
            setMotors(0, 0);
            Serial.println("[DONE] Max correction reached, stop to avoid oscillation");
            break;
        }

        float P = Kp_rot * error;
        float D = Kd_rot * (error - lastError) / dt;
        lastError = error;

        int speed = (int)(P + D);
        if (speed > 0) speed = constrain(speed, min_turn_speed, max_turn_speed);
        else           speed = constrain(speed, -max_turn_speed, -min_turn_speed);

        setMotors(-speed, speed);

        if (millis() - startTime > 5000) {
            setMotors(0, 0);
            Serial.println("[ERROR] Timeout!");
            break;
        }
        delay(20); // Tăng delay lên 20ms để cơ khí kịp phản ứng
    }
}

// ================= SETUP & LOOP TEST =================

void setup() {
    Serial.begin(115200);
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    Wire.begin(21, 22);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
    
    Serial.println("Calibrating Gyro...");
    long sum = 0;
    for(int i=0; i<600; i++) {
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) sum += (Wire.read() << 8) | Wire.read();
        delay(2);
    }
    gyroOffsetZ = (float)sum / 600.0f;
    lastGyroMs = millis();
    delay(1000);
}

void loop() {
    rotateToAnglePID(90.0);
    delay(3000);
    rotateToAnglePID(0.0);
    delay(3000);
}
