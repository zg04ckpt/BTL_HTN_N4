#include "encoder.h"
#include <Arduino.h>

// PIN
const int ENCODER_LEFT_PIN = 34;
const int ENCODER_RIGHT_PIN = 35;

// Biến
volatile long leftPulses = 0;
volatile long rightPulses = 0;

// Setup pin cho Encoder
void setupPINForEncoder() {
    pinMode(ENCODER_LEFT_PIN, INPUT);
    digitalWrite(ENCODER_LEFT_PIN, LOW);
    pinMode(ENCODER_RIGHT_PIN, INPUT);
    digitalWrite(ENCODER_RIGHT_PIN, LOW);
}

// Tăng số xung cho Encoder
void IRAM_ATTR incLeftPulses() { leftPulses++; }
void IRAM_ATTR incRightPulses() { rightPulses++; }

// Attach Interrupt cho Encoder khi có xung tăng lên (LOW -> HIGH = RISING)
// Khi có tín hiệu thì ngắt này sẽ được gọi để tránh làm mất xung khi đang xử lý
void attachEncoderInterrupt() {
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), incLeftPulses, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), incRightPulses, RISING);
}

// Setup tất cả cấu hình cho module Encoder
void setupEncoder() {
    setupPINForEncoder();
    attachEncoderInterrupt();
}