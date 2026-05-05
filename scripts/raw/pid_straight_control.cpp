#include <Arduino.h>
#include <Wire.h>

const int ENA = 13, IN1 = 12, IN2 = 14;
const int ENB = 25, IN3 = 27, IN4 = 26;
const uint8_t MPU_ADDR = 0x68;
const float GYRO_SCALE = 131.0f;

float yawDeg = 0, gyroOffsetZ = 0;
unsigned long lastGyroMs = 0, lastPIDTime = 0;
float targetAngle = 0, lastError = 0, dFiltered = 0;
bool firstRun = true;

// --- BIẾN ĐIỀU KHIỂN TỐC ĐỘ ---
float currentSpeed = 0;
const float MAX_SPEED = 85.0f; // Tốc độ mục tiêu tối đa
const float RAMP_STEP = 0.4f;  // Độ mượt khi tăng tốc (càng nhỏ càng mượt)
const int MOTOR_DEADZONE_PWM = 35; // PWM tối thiểu để motor bắt đầu quay ổn định

// --- THÔNG SỐ CÂN BẰNG ĐỘNG CƠ ---
const float LEFT_GAIN  = 1.0f;
const float RIGHT_GAIN = 0.8f; // Đồng bộ theo bản đã cân chỉnh ổn định

// --- THÔNG SỐ PID ---
float Kp = 1.5f; 
float Kd = 0.2f;
const float D_FILTER_ALPHA = 0.75f;

float normalizeAngle180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void setMotors(int speedL, int speedR) {
    // Áp dụng Gain để bù trừ sự lệch nhau của motor
    int leftCmd = (int)(speedL * LEFT_GAIN);
    int rightCmd = -(int)(speedR * RIGHT_GAIN); 
    
    // Bánh Trái
    if (leftCmd >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
    else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); leftCmd = -leftCmd; }
    if (leftCmd > 0 && leftCmd < MOTOR_DEADZONE_PWM) leftCmd = MOTOR_DEADZONE_PWM;
    analogWrite(ENA, constrain(leftCmd, 0, 150));

    // Bánh Phải
    if (rightCmd >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
    else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); rightCmd = -rightCmd; }
    if (rightCmd > 0 && rightCmd < MOTOR_DEADZONE_PWM) rightCmd = MOTOR_DEADZONE_PWM;
    analogWrite(ENB, constrain(rightCmd, 0, 150));
}

void updateYaw() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return;
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    
    unsigned long now = millis();
    float dt = (now - lastGyroMs) / 1000.0f;
    lastGyroMs = now;
    if (dt <= 0 || dt > 0.1f) return;
    
    yawDeg += ((rawZ - gyroOffsetZ) / GYRO_SCALE) * dt;
}

void driveStraightPID(float targetSpeed, float targetYaw) {
    unsigned long now = millis();
    float dt = (now - lastPIDTime) / 1000.0f;
    if (dt <= 0) return;
    lastPIDTime = now;

    float error = normalizeAngle180(targetYaw - yawDeg);

    float dRaw = (error - lastError) / dt;
    dFiltered = D_FILTER_ALPHA * dFiltered + (1.0f - D_FILTER_ALPHA) * dRaw;
    float adjustment = (Kp * error) + (Kd * dFiltered);
    lastError = error;

    // Giới hạn bẻ lái theo tốc độ hiện tại:
    // Mục tiêu là KHÔNG cho tốc độ 2 bánh đổi dấu (vì đổi dấu => đổi chiều motor => nhìn như quay xen kẽ).
    float maxAdjustBySpeed = targetSpeed * 0.45f;
    if (maxAdjustBySpeed > 40.0f) maxAdjustBySpeed = 40.0f;
    if (maxAdjustBySpeed < 0.0f) maxAdjustBySpeed = 0.0f;
    adjustment = constrain(adjustment, -maxAdjustBySpeed, maxAdjustBySpeed);

    float leftCmdF = targetSpeed + adjustment;
    float rightCmdF = targetSpeed - adjustment;
    if (leftCmdF < 0.0f) leftCmdF = 0.0f;
    if (rightCmdF < 0.0f) rightCmdF = 0.0f;
    if (leftCmdF > MAX_SPEED) leftCmdF = MAX_SPEED;
    if (rightCmdF > MAX_SPEED) rightCmdF = MAX_SPEED;

    setMotors((int)leftCmdF, (int)rightCmdF);
}

void setup() {
    Serial.begin(115200);
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    Wire.begin(21, 22);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
    
    Serial.println("Dang Calibrate... Hay giu robot dung yen!");
    long sum = 0;
    for(int i=0; i<600; i++) {
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) sum += (Wire.read() << 8) | Wire.read();
        delay(2);
    }
    gyroOffsetZ = (float)sum / 600.0f;
    lastGyroMs = lastPIDTime = millis();
    Serial.println("Xong! Cho 1s de bat dau di thang...");
    delay(1000);
}

void loop() {
    updateYaw();
    
    // 1. LƯU GÓC MỤC TIÊU KHI BẮT ĐẦU CHẠY
    if (firstRun) {
        targetAngle = yawDeg;
        firstRun = false;
        Serial.print("Goc muc tieu: "); Serial.println(targetAngle);
    }
    
    // 2. TĂNG TỐC DẦN (RAMP-UP)
    if (currentSpeed < MAX_SPEED) {
        currentSpeed += RAMP_STEP;
    }

    // 3. ĐI THẲNG VÀ SỬA LỖI HƯỚNG
    driveStraightPID(currentSpeed, targetAngle);
    
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 200) {
        Serial.print("Yaw: "); Serial.print(yawDeg);
        Serial.print(" | Error: "); Serial.print(targetAngle - yawDeg);
        Serial.print(" | Speed: "); Serial.println(currentSpeed);
        lastLog = millis();
    }
    delay(10);
}

