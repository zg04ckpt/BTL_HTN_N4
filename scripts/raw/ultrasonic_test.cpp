#include <Arduino.h>

// Ultrasonic sensor test (HC-SR04 style)
// Wiring requested: TRIG -> GPIO33, ECHO -> GPIO32
const int TRIG_PIN = 33;
const int ECHO_PIN = 32;

unsigned long lastReadMs = 0;

float readDistanceCm() {
  // Ensure a clean LOW pulse before sending TRIG HIGH.
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Timeout ~30 ms (about 5 meters max range).
  unsigned long durationUs = pulseIn(ECHO_PIN, HIGH, 30000);
  if (durationUs == 0) {
    return -1.0f; // Timeout / no echo.
  }

  return (durationUs * 0.0343f) / 2.0f;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.println();
  Serial.println("=== ULTRASONIC TEST START ===");
  Serial.println("TRIG=GPIO33, ECHO=GPIO32");
}

void loop() {
  if (millis() - lastReadMs >= 500) {
    float distanceCm = readDistanceCm();

    if (distanceCm < 0) {
      Serial.println("No echo (timeout)");
    } else {
      Serial.print("Distance: ");
      Serial.print(distanceCm, 1);
      Serial.println(" cm");
    }

    lastReadMs = millis();
  }
}
