#include <Arduino.h>
#include <Wire.h>
#include <math.h>

const int LOOP_DELAY_MS = 20;
const float WHEEL_R = 3.4f; //cm
const float TOLERANCE = 1.0f; //độ sai số góc cho phép
const int ERROR_TIME_OUT = 5000; //Thời gian tối đã để thực hiện 1 hành động, vượt quá => Lỗi
volatile long leftPulses = 0; // Số xung đếm được bên trái
volatile long rightPulses = 0; // Số xung đếm được bên phải
volatile float yawDeg = 0; // Góc quay hiện tại

#pragma region L298N + MOVING
const int ENA = 13, IN1 = 12, IN2 = 14;
const int ENB = 25, IN3 = 27, IN4 = 26;
const float LEFT_GAIN = 0.985f;
const float RIGHT_GAIN = 1.0f;
const int MIN_PWM = 80;
const int MAX_PWM = 100;

void setupPINForL298N() {
    pinMode(ENA, OUTPUT); 
    pinMode(IN1, OUTPUT); 
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); 
    pinMode(IN3, OUTPUT); 
    pinMode(IN4, OUTPUT);
}

void go(int speed) {
    executeControl(speed * LEFT_GAIN, speed * RIGHT_GAIN);
}

void rotate(float targetDeg) {
    unsigned long startTime = millis();
    while (true) {
        updateYaw();

        float currentError = targetDeg - yawDeg;

        // Dừng
        if (abs(currentError) <= TOLERANCE) {
            stop();
            return;
        }

        // Quay thất bại
        if (millis() - startTime > ERROR_TIME_OUT) {
            stop();
            Serial.println("Rotate TIMEOUT!");
            break;
        }

        // Khi góc càng nhỏ quay chậm để tránh bị quán tính làm lệch
        int speed = 80;

        // Xác định hướng quay
        if (currentError > 0) {
            // Quay phải
            executeControl(-speed, speed);
        } else {
            // Quay trái
            executeControl(speed, -speed);
        }

        delay(100);
    }
}

void executeControl(int speedL, int speedR) {
    int leftCmd = speedL;
    int rightCmd = speedR; // Đảo chiều bánh phải theo thiết kế của bạn

    // Logic hướng Motor A
    if (leftCmd >= 0) {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
        leftCmd = -leftCmd;
    }

    // Logic hướng Motor B
    if (rightCmd >= 0) {
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
        rightCmd = -rightCmd;
    }

    // Xử lý Deadzone
    if (leftCmd > 0 && leftCmd < MIN_PWM) leftCmd = MIN_PWM;
    if (rightCmd > 0 && rightCmd < MIN_PWM) rightCmd = MIN_PWM;

    // Xuất PWM
    analogWrite(ENA, constrain(leftCmd, 0, MAX_PWM));
    analogWrite(ENB, constrain(rightCmd, 0, MAX_PWM));
}

void stop() {
  // BƯỚC 1: PHANH GẤP (Active Braking)
  // Đưa các chân IN về cùng mức LOW và bật Enable lên mức tối đa
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, 255); 
  analogWrite(ENB, 255);

  // BƯỚC 2: DUY TRÌ LỰC PHANH
  delay(150); 

  // BƯỚC 3: NGẮT ĐIỆN HOÀN TOÀN (Power Down)
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
#pragma endregion

#pragma region MPU
const uint8_t MPU_ADDR = 0x68;
const float GYRO_SCALE = 131.0f;
float gyroOffsetZ = 0;
unsigned long lastGyroMs = 0;

void setupPinForMPU() {
    Wire.begin(21, 22);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
}

void setupMPUVariableInitValues() {
    long sum = 0;
    Serial.println("Calibrating... Keep robot still!");
    
    for(int i=0; i<600; i++) {
        Wire.beginTransmission(MPU_ADDR); 
        Wire.write(0x47); 
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, (uint8_t)2);
        
        if (Wire.available() >= 2) {
            int16_t val = (Wire.read() << 8) | Wire.read();
            sum += val;
        }
        delay(3); // Tăng nhẹ delay để cảm biến kịp cập nhật mẫu mới
    }
    
    gyroOffsetZ = (float)sum / 600.0f; // Lưu Offset ở dạng Raw
    
    // Đợi một chút để mọi thứ ổn định
    delay(100); 
    // Gán lastGyroMs cuối cùng để dt lần đầu tiên thật nhỏ
    lastGyroMs = millis();
    yawDeg = 0; 
    Serial.println("Done!");
}

void updateYaw() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)2);
    
    if (Wire.available() < 2) return;
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    
    unsigned long now = millis();
    // Tính Delta T (giây)
    float dt = (now - lastGyroMs) / 1000.0f;
    lastGyroMs = now;
    
    // Chặn nhiễu thời gian
    if (dt <= 0 || dt > 0.5f) return; 

    // Tính vận tốc góc (đã trừ offset và chia tỉ lệ)
    float gyroRateZ = (rawZ - gyroOffsetZ) / GYRO_SCALE;
    
    // Cộng dồn vào góc tổng
    // Bạn có thể thêm một "Deadzone" nhỏ để triệt tiêu trôi góc hoàn toàn
    if (abs(gyroRateZ) > 0.1) { // Chỉ cộng nếu vận tốc góc > 0.1 độ/s
        yawDeg += gyroRateZ * dt;
    }
}

void debugMPU() {
    static unsigned long lastDebugMs = 0;
    
    // Chỉ in dữ liệu mỗi 100ms để tránh làm nghẽn Serial và ảnh hưởng tốc độ loop
    if (millis() - lastDebugMs > 100) {
        lastDebugMs = millis();

        // Tính vận tốc góc tức thời để xem độ nhiễu (dùng lại logic trong updateYaw)
        // Lưu ý: rawZ ở đây là giá trị bạn vừa đọc từ thanh ghi 0x47
        // float currentRate = (rawZ - gyroOffsetZ) / GYRO_SCALE;

        Serial.print("Yaw:");
        Serial.print(yawDeg);         // Góc hiện tại (Độ)
        
        Serial.print("\t");           // Khoảng cách Tab
        
        // In thêm một đường cơ sở 0 để dễ quan sát trên Serial Plotter
        Serial.print("Base:0"); 
        
        Serial.println(); 
    }
}
#pragma endregion

#pragma region Encoder
const int ENCODER_LEFT_PIN = 34;
const int ENCODER_RIGHT_PIN = 35;


void setupPINForEncoder() {
    pinMode(ENCODER_LEFT_PIN, INPUT);
    digitalWrite(ENCODER_LEFT_PIN, LOW);
    pinMode(ENCODER_RIGHT_PIN, INPUT);
    digitalWrite(ENCODER_RIGHT_PIN, LOW);
}

void IRAM_ATTR incLeftPulses() {
    leftPulses++;
}

void IRAM_ATTR incRightPulses() {
    rightPulses++;
}

// Khi encoder đếm được xung thì (LOW -> HIGH = RISING)
void attachEncoderInterrupt() {
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), incLeftPulses, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), incRightPulses, RISING);
}

void debugEncoder() {
    // 1. Tạm thời tắt ngắt để copy giá trị an toàn
    noInterrupts(); 
    long tempLeft = leftPulses;
    long tempRight = rightPulses;
    interrupts(); // Bật lại ngắt ngay lập tức

    // 2. In giá trị ra Serial Monitor
    Serial.print("Encoder L: ");
    Serial.print(tempLeft);
    Serial.print(" \t| "); // Dùng \t để căn lề cho thẳng hàng
    Serial.print("Encoder R: ");
    Serial.println(tempRight);
}

// --- Điều khiển theo xung encoder + PID cân 2 bánh (sai số = dL - dR) ---
// Mục tiêu đi thẳng: (dL+dR)/2 -> targetPulses, đồng thời dL ~ dR.
// Mục tiêu quay tại chỗ: (dL+dR)/2 -> target (tổng xung hai bánh), đồng thời dL ~ dR.
const long ENC_STRAIGHT_TARGET_PULSES = 400;   // chỉnh theo quãng đường mong muốn
const long ENC_TURN_TARGET_PULSES = 120;       // chỉnh theo góc quay (thử nghiệm)
const long ENC_PULSE_DIST_TOL = 4;             // dừng khi thiếu xung còn lại <= này
const long ENC_PULSE_BAL_TOL = 5;             // |dL-dR| chấp nhận được

const float ENC_DIST_KP = 0.45f;               // PWM cơ bản theo lỗi quãng xung trung bình
const float ENC_BAL_KP = 0.35f;                // bù theo lệch xung hai bánh
const float ENC_BAL_KD = 0.08f;

void encoderCountsSnapshot(long &outL, long &outR) {
    noInterrupts();
    outL = leftPulses;
    outR = rightPulses;
    interrupts();
}

void encoderResetZero() {
    noInterrupts();
    leftPulses = 0;
    rightPulses = 0;
    interrupts();
}

/**
 * Đi thẳng đúng số xung (mỗi bánh tăng ~targetPulses), PID cân bằng: balErr = dL - dR.
 */
void driveStraightPulsePID(long targetPulses) {
    encoderResetZero();
    unsigned long t0 = millis();
    unsigned long lastPidMs = millis();
    float lastBalErr = 0.0f;

    Serial.print("[STR_ENC] target pulses (avg): ");
    Serial.println(targetPulses);

    while (true) {
        long dL = 0, dR = 0;
        encoderCountsSnapshot(dL, dR);
        float avg = (float)(dL + dR) * 0.5f;
        float distErr = (float)targetPulses - avg;
        float balErr = (float)(dL - dR);

        if (distErr <= (float)ENC_PULSE_DIST_TOL && fabs(balErr) <= (float)ENC_PULSE_BAL_TOL) {
            stop();
            Serial.print("[STR_ENC] done avg=");
            Serial.print(avg, 1);
            Serial.print(" bal=");
            Serial.println(balErr, 1);
            return;
        }

        if ((int)(millis() - t0) > ERROR_TIME_OUT) {
            stop();
            Serial.println("[STR_ENC] TIMEOUT");
            return;
        }

        unsigned long now = millis();
        float dt = (now - lastPidMs) / 1000.0f;
        if (dt <= 0.0f || dt > 0.25f) dt = 0.02f;
        lastPidMs = now;

        float base = ENC_DIST_KP * distErr;
        if (base > (float)MAX_PWM) base = (float)MAX_PWM;
        if (base < (float)MIN_PWM) base = (float)MIN_PWM;

        float dBal = (balErr - lastBalErr) / dt;
        lastBalErr = balErr;
        float adj = ENC_BAL_KP * balErr + ENC_BAL_KD * dBal;
        if (adj > 25.0f) adj = 25.0f;
        if (adj < -25.0f) adj = -25.0f;

        float leftSpd = base - adj;
        float rightSpd = base + adj;
        if (leftSpd < (float)MIN_PWM) leftSpd = (float)MIN_PWM;
        if (rightSpd < (float)MIN_PWM) rightSpd = (float)MIN_PWM;
        if (leftSpd > (float)MAX_PWM) leftSpd = (float)MAX_PWM;
        if (rightSpd > (float)MAX_PWM) rightSpd = (float)MAX_PWM;

        executeControl((int)(leftSpd * LEFT_GAIN), (int)(rightSpd * RIGHT_GAIN));
        delay(LOOP_DELAY_MS);
    }
}

/**
 * Quay tại chỗ: cả hai bánh đều tăng xung (encoder 1 kênh), tiến độ dùng (dL+dR)/2.
 * ccw=true: trái lùi, phải tiến (cùng kiểu rotate() khi currentError < 0 trong code cũ).
 */
void rotateInPlacePulsePID(long targetPulsesHalfSum, bool ccw) {
    encoderResetZero();
    unsigned long t0 = millis();
    unsigned long lastPidMs = millis();
    float lastBalErr = 0.0f;

    Serial.print("[ROT_ENC] target (dL+dR)/2: ");
    Serial.println(targetPulsesHalfSum);

    while (true) {
        long dL = 0, dR = 0;
        encoderCountsSnapshot(dL, dR);
        float progress = (float)(dL + dR) * 0.5f;
        float distErr = (float)targetPulsesHalfSum - progress;
        float balErr = (float)(dL - dR);

        if (distErr <= (float)ENC_PULSE_DIST_TOL && fabs(balErr) <= (float)ENC_PULSE_BAL_TOL) {
            stop();
            Serial.print("[ROT_ENC] done progress=");
            Serial.print(progress, 1);
            Serial.print(" bal=");
            Serial.println(balErr, 1);
            return;
        }

        if ((int)(millis() - t0) > ERROR_TIME_OUT) {
            stop();
            Serial.println("[ROT_ENC] TIMEOUT");
            return;
        }

        unsigned long now = millis();
        float dt = (now - lastPidMs) / 1000.0f;
        if (dt <= 0.0f || dt > 0.25f) dt = 0.02f;
        lastPidMs = now;

        float base = ENC_DIST_KP * distErr;
        if (base > (float)MAX_PWM) base = (float)MAX_PWM;
        if (base < (float)MIN_PWM) base = (float)MIN_PWM;

        float dBal = (balErr - lastBalErr) / dt;
        lastBalErr = balErr;
        float adj = ENC_BAL_KP * balErr + ENC_BAL_KD * dBal;
        if (adj > 25.0f) adj = 25.0f;
        if (adj < -25.0f) adj = -25.0f;

        float magL = base - adj;
        float magR = base + adj;
        if (magL < (float)MIN_PWM) magL = (float)MIN_PWM;
        if (magR < (float)MIN_PWM) magR = (float)MIN_PWM;
        if (magL > (float)MAX_PWM) magL = (float)MAX_PWM;
        if (magR > (float)MAX_PWM) magR = (float)MAX_PWM;

        int lCmd = (int)(magL * LEFT_GAIN);
        int rCmd = (int)(magR * RIGHT_GAIN);
        if (ccw) {
            executeControl(-lCmd, rCmd);
        } else {
            executeControl(lCmd, -rCmd);
        }
        delay(LOOP_DELAY_MS);
    }
}

#pragma endregion

// Bật 1 để chạy demo đi thẳng + quay theo xung; tắt 0 để giữ loop MPU như cũ
#ifndef FINAL2_USE_ENCODER_PID_DEMO
#define FINAL2_USE_ENCODER_PID_DEMO 1
#endif

void setup() {
    Serial.begin(115200);

    setupPINForL298N();

    setupPINForEncoder();
    attachEncoderInterrupt();

    setupPinForMPU();                    // ← đã có
    setupMPUVariableInitValues();
}

void loop() {
    updateYaw();
    debugMPU();

    if (abs(yawDeg) > TOLERANCE) {
        rotate(0);
    }

    delay(LOOP_DELAY_MS);
}
