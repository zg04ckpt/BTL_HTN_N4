// ========================================
// TEST: ZIGZAG NAVIGATION
// Logic: Đi thẳng -> Gặp tường -> Quay 90° -> Dịch 20cm -> Quay 90° -> Đi thẳng
// Luân phiên quay trái/phải mỗi lần gặp tường
// ========================================

#include <Arduino.h>
#include <Wire.h>

// Pin map from main.cpp
const int ENA = 13, IN1 = 12, IN2 = 14;
const int ENB = 25, IN3 = 27, IN4 = 26;
const int TRIG_PIN = 33;
const int ECHO_PIN = 32;
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const uint8_t MPU_ADDR = 0x68;
const int SIGNAL_PIN = 19;
const int LINE_PIN = 39; // SVN

const bool LEFT_MOTOR_REVERSED = false;
const bool RIGHT_MOTOR_REVERSED = true;

const int GO_SPEED_LEFT = 80;      // Tốc độ đi thẳng (tăng từ 50)
const int GO_SPEED_RIGHT = 80;
const int TURN_SPEED_LEFT = 90;    // Tốc độ quay (tăng từ 50)
const int TURN_SPEED_RIGHT = 90;
const int LEFT_TRIM = 0;
const int RIGHT_TRIM = 0;
const int MAX_PWM = 120;           // Giới hạn PWM tối đa

const float WALL_DISTANCE_CM = 20.0f;           // Khoảng cách phát hiện tường
const unsigned long DIST_READ_INTERVAL_MS = 100; // Kiểm tra sensor mỗi 100ms
const unsigned long TURN_TIMEOUT_MS = 5000;      // Timeout quay 90° (5s)
const float TURN_ANGLE_DEG = 90.0f;              // Góc quay mỗi lần (90°)
const float TURN_TOLERANCE_DEG = 5.0f;           // Sai số cho phép
const float SHIFT_DISTANCE_CM = 20.0f;           // Độ dịch ngang mỗi lần
const unsigned long SHIFT_DURATION_MS = 1000;    // Thời gian dịch (ước tính)
const float KP_TURN = 1.0f;
const float KD_TURN = 0.5f;
const int MIN_TURN_SPEED = 28;
const int MAX_TURN_SPEED = 90;
const unsigned long TURN_RAMP_UP_MS = 180;
const int TURN_MID_SPEED = 60;
const int TURN_NEAR_SPEED = 40;
const int TURN_FINAL_SPEED = 25;

const int CALIB_SAMPLES = 500;
const float GYRO_SCALE = 131.0f; // LSB/(deg/s) for +/-250 dps

float gyroOffsetZ = 0.0f;
float yawDeg = 0.0f;
unsigned long lastGyroMs = 0;

unsigned long lastDistMs = 0;
unsigned long lastLogMs = 0;

enum RobotMode {
  MODE_FORWARD,     // Đi thẳng cho đến khi gặp tường
  MODE_SHIFT,       // Dịch ngang 20cm (giữa 2 lần quay blocking)
  MODE_COOLDOWN     // Dừng ngắn để ổn định
};

RobotMode mode = MODE_FORWARD;
unsigned long cooldownStartMs = 0;
int zigzagRowCount = 0;       // Đếm số hàng đã đi (quyết định target angle ở TURN2)
unsigned long shiftStartMs = 0;

float referenceYaw = 0.0f;   // Góc tham chiếu lưu ở lần chạy đầu tiên (A)
bool isFirstRun = true;      // Cờ để biết có phải lần chạy đầu tiên hay không
float targetAngle = 0.0f;    // Góc mục tiêu cho mỗi lần quay

void finishZigzagSequence();

void setMotors(int speedL, int speedR) {
  int leftCmd = LEFT_MOTOR_REVERSED ? -speedL : speedL;
  int rightCmd = RIGHT_MOTOR_REVERSED ? -speedR : speedR;

  if (leftCmd >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftCmd = -leftCmd;
  }
  analogWrite(ENA, constrain(leftCmd, 0, MAX_PWM));

  if (rightCmd >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightCmd = -rightCmd;
  }
  analogWrite(ENB, constrain(rightCmd, 0, MAX_PWM));
}

void stopMotors() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  delay(120);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long durationUs = pulseIn(ECHO_PIN, HIGH, 30000);
  if (durationUs == 0) {
    return -1.0f;
  }

  return (durationUs * 0.0343f) / 2.0f;
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

int16_t mpuRead16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
  return (Wire.read() << 8) | Wire.read();
}

void calibrateGyroZ() {
  long sum = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    sum += mpuRead16(0x47);
    delay(3);
  }
  gyroOffsetZ = (float)sum / (float)CALIB_SAMPLES;
}

void updateYaw() {
  int16_t rawZ = mpuRead16(0x47);
  unsigned long now = millis();
  float dt = (now - lastGyroMs) / 1000.0f;
  lastGyroMs = now;

  if (dt <= 0.0f || dt > 0.2f) {
    return;
  }

  float gyroZ = (rawZ - gyroOffsetZ) / GYRO_SCALE;
  yawDeg += gyroZ * dt;
}

float normalizeAngle180(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }
  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }
  return angleDeg;
}

int computeTurnSpeedProfile(float absError, unsigned long elapsedMs) {
  int profileMax = MAX_TURN_SPEED;
  if (absError < 40.0f) profileMax = TURN_MID_SPEED;
  if (absError < 20.0f) profileMax = TURN_NEAR_SPEED;
  if (absError < 10.0f) profileMax = TURN_FINAL_SPEED;

  int rampMax = map((long)constrain((long)elapsedMs, 0L, (long)TURN_RAMP_UP_MS), 0L, (long)TURN_RAMP_UP_MS, MIN_TURN_SPEED, MAX_TURN_SPEED);
  return min(profileMax, rampMax);
}

void executeTurnToTarget(const __FlashStringHelper* turnTag, float target) {
  targetAngle = normalizeAngle180(target);
  unsigned long startMs = millis();
  unsigned long lastTime = millis();
  float lastError = 0.0f;

  Serial.print(turnTag);
  Serial.print(F(" Starting turn to "));
  Serial.print(targetAngle);
  Serial.println(F(" deg"));

  while (true) {
    updateYaw();
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0.0f) {
      continue;
    }
    lastTime = now;

    float error = normalizeAngle180(targetAngle - yawDeg);
    float absError = fabsf(error);

    if (absError <= TURN_TOLERANCE_DEG) {
      stopMotors();
      Serial.print(turnTag);
      Serial.print(F(" COMPLETE target="));
      Serial.print(targetAngle, 1);
      Serial.print(F(" current="));
      Serial.println(yawDeg, 1);
      break;
    }

    float p = KP_TURN * error;
    float d = KD_TURN * (error - lastError) / dt;
    lastError = error;

    int pidSpeed = (int)(p + d);
    int speedLimit = computeTurnSpeedProfile(absError, now - startMs);
    int speedMag = constrain(abs(pidSpeed), MIN_TURN_SPEED, speedLimit);
    int speed = (error >= 0.0f) ? speedMag : -speedMag;
    setMotors(-speed, speed);

    if (now - lastLogMs >= 500) {
      Serial.print(turnTag);
      Serial.print(F(" Current="));
      Serial.print(yawDeg, 1);
      Serial.print(F(" Target="));
      Serial.print(targetAngle, 1);
      Serial.print(F(" Error="));
      Serial.println(error, 1);
      lastLogMs = now;
    }

    if (millis() - startMs >= TURN_TIMEOUT_MS) {
      stopMotors();
      Serial.print(turnTag);
      Serial.println(F(" TIMEOUT"));
      break;
    }
    delay(20);
  }
  delay(300);
}

// Blocking function: Quay đến góc chính xác (A - 90°)
void executeTurn1() {
  executeTurnToTarget(F("[TURN1]"), referenceYaw - TURN_ANGLE_DEG);
}

// Blocking function: Quay đến góc chính xác (A - 180° hoặc A)
void executeTurn2() {
  // Tính target angle dựa trên row count
  float target = referenceYaw;
  if (zigzagRowCount % 2 == 0) {
    target = referenceYaw - (2.0f * TURN_ANGLE_DEG);
    Serial.print(F("[TURN2] Next target A - 180 = "));
  } else {
    target = referenceYaw;
    Serial.print(F("[TURN2] Next target A = "));
  }
  Serial.print(normalizeAngle180(target));
  Serial.print(F(" deg (row "));
  Serial.print(zigzagRowCount);
  Serial.println(F(")"));
  executeTurnToTarget(F("[TURN2]"), target);
}

void beginZigzagTurn() {
  stopMotors();
  delay(300);
  
  // Lưu góc tham chiếu ở lần chạy đầu tiên
  if (isFirstRun) {
    referenceYaw = yawDeg;
    isFirstRun = false;
    Serial.print(F("[REF YAW] Saved reference angle (A): "));
    Serial.print(referenceYaw);
    Serial.println(F("°"));
  }
  
  // Gọi hàm quay blocking
  executeTurn1();
  
  // Sau khi quay xong, chuyển sang shift
  delay(200);
  shiftStartMs = millis();
  mode = MODE_SHIFT;
}

void beginShift() {
  stopMotors();
  delay(200);
  shiftStartMs = millis();
  mode = MODE_SHIFT;
  Serial.println(F("[SHIFT] Moving sideways 20cm"));
}

void beginTurn2() {
  stopMotors();
  delay(200);
  
  // Gọi hàm quay blocking
  executeTurn2();
  
  // Sau khi quay xong, hoàn thành zigzag sequence
  finishZigzagSequence();
}

void finishZigzagSequence() {
  stopMotors();
  cooldownStartMs = millis();
  mode = MODE_COOLDOWN;
  zigzagRowCount++;  // Tăng số hàng
  Serial.print(F("[COMPLETE] Zigzag row #"));
  Serial.print(zigzagRowCount);
  Serial.println(F(" finished"));
}

void isHome() {
  
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(LINE_PIN, INPUT);
  pinMode(SIGNAL_PIN, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  mpuWrite(0x6B, 0x00);
  delay(50);

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("      TEST: ZIGZAG NAVIGATION           "));
  Serial.println(F("========================================"));
  Serial.println(F("Keep robot still for gyro calibration..."));

  calibrateGyroZ();
  lastGyroMs = millis();

  Serial.print(F("GyroZ offset: "));
  Serial.println(gyroOffsetZ, 2);
  Serial.println();
  Serial.println(F("Test Config:"));
  Serial.print(F("  - Wall distance: "));
  Serial.print(WALL_DISTANCE_CM);
  Serial.println(F(" cm"));
  Serial.print(F("  - Go speed: "));
  Serial.println(GO_SPEED_LEFT);
  Serial.print(F("  - Turn speed: "));
  Serial.println(TURN_SPEED_LEFT);
  Serial.println(F(" deg"));
  Serial.print(F("  - Shift distance: "));
  Serial.print(SHIFT_DISTANCE_CM);
  Serial.println(F(" cm"));
  Serial.println();
  Serial.println(F("Zigzag pattern: Forward -> Turn 90 -> Shift -> Turn 90 -> Forward"));
  Serial.println(F("Starting test in 2 seconds..."));

  stopMotors();
  delay(2000);
}

void loop() {
  updateYaw();
  unsigned long now = millis();

  if (mode == MODE_FORWARD) {
    // Đi thẳng và kiểm tra vật cản
    int leftOut = constrain(GO_SPEED_LEFT + LEFT_TRIM, 0, MAX_PWM);
    int rightOut = constrain(GO_SPEED_RIGHT + RIGHT_TRIM, 0, MAX_PWM);
    setMotors(leftOut, rightOut);

    if (now - lastDistMs >= DIST_READ_INTERVAL_MS) {
      float dist = readDistanceCm();
      if (dist > 0 && dist <= WALL_DISTANCE_CM) {
        Serial.print(F("[WALL DETECTED] Distance: "));
        Serial.print(dist);
        Serial.println(F(" cm - Starting zigzag turn"));
        beginZigzagTurn();
      }
      lastDistMs = now;
    }
    
  } else if (mode == MODE_SHIFT) {
    // Dịch ngang 20cm (đi thẳng trong thời gian ngắn)
    int leftOut = constrain(GO_SPEED_LEFT + LEFT_TRIM, 0, MAX_PWM);
    int rightOut = constrain(GO_SPEED_RIGHT + RIGHT_TRIM, 0, MAX_PWM);
    setMotors(leftOut, rightOut);

    if (now - shiftStartMs >= SHIFT_DURATION_MS) {
      stopMotors();
      Serial.println(F("[SHIFT COMPLETE]"));
      delay(200);
      beginTurn2();  // Gọi turn2 blocking function
    }
    
  } else if (mode == MODE_COOLDOWN) {
    // Dừng ngắn sau khi hoàn thành zigzag để ổn định
    if (now - cooldownStartMs >= 500) {
      Serial.println(F("[FORWARD] Resuming straight movement"));
      mode = MODE_FORWARD;
    }
  }

  if (now - lastLogMs >= 500) {
    Serial.print(F("Mode="));
    switch(mode) {
      case MODE_FORWARD: Serial.print(F("FORWARD")); break;
      case MODE_SHIFT: Serial.print(F("SHIFT")); break;
      case MODE_COOLDOWN: Serial.print(F("COOLDOWN")); break;
    }
    Serial.print(F(" | Yaw="));
    Serial.print(yawDeg, 1);
    Serial.print(F("° | Row="));
    Serial.println(zigzagRowCount);
    lastLogMs = now;
  }

  delay(5);
}
