#include "l298n.h"
#include "../../config.h"

// PIN
const int ENA = 13;
const int IN1 = 12;
const int IN2 = 14;
const int ENB = 25;
const int IN3 = 27;
const int IN4 = 26;

// Setup pin cho L298N
void setupPINForL298N() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

// Setup tất cả cấu hình cho module L298N
void setupL298N() {
    setupPINForL298N();
}

// Điều khiển trực tiếp 2 bánh
// - pwmL/pwmR: 0..255
// - forwardL/forwardR: true = tiến, false = lùi
void setMotor(int pwmL, bool forwardL, int pwmR, bool forwardR) {

    // Chỉnh lệch cơ khí
    // pwmL = (int)(pwmL * LEFT_GAIN);
    // pwmR = (int)(pwmR * RIGHT_GAIN);

    // Chuẩn hóa phạm vi
    pwmL = constrain(pwmL, 0, 255);
    pwmR = constrain(pwmR, 0, 255);
  
    if (pwmL <= 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
    } else {
      digitalWrite(IN1, forwardL ? HIGH : LOW);
      digitalWrite(IN2, forwardL ? LOW : HIGH);
      analogWrite(ENA, pwmL);
    }
  
    if (pwmR <= 0) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 0);
    } else {
      digitalWrite(IN3, forwardR ? HIGH : LOW);
      digitalWrite(IN4, forwardR ? LOW : HIGH);
      analogWrite(ENB, pwmR);
    }
}

void stopMotor() {
    // BƯỚC 1: phanh gấp (active braking)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);

    // BƯỚC 2: giữ lực phanh trong thời gian ngắn
    delay(150);

    // BƯỚC 3: ngắt điện hoàn toàn
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}