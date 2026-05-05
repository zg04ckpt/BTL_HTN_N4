#include "ultrasonic.h"

// PIN
const int TRIG_PIN = 33;
const int ECHO_PIN = 32;

// Setup pin cho Ultrasonic
void setupPINForUltrasonic() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
}

// Setup tất cả cấu hình cho module Ultrasonic
void setupUltrasonic() {
    setupPINForUltrasonic();
}

// Đọc khoảng cách phía trước (cm)
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
  