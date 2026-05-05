#include <Arduino.h>

// Encoder test with current hardware pin mapping (NOT the reference example pins)
const int ENCODER_LEFT_PIN = 34;
const int ENCODER_RIGHT_PIN = 35;

volatile long leftPulseCount = 0;
volatile long rightPulseCount = 0;

// Set according to your encoder spec (pulses per one wheel revolution).
const int PULSES_PER_REV = 20;

unsigned long lastLogMs = 0;

void setupEncoderInputPin(int pin) {
  // ESP32 GPIO34..GPIO39 are input-only and do not support internal pull-up.
  if (pin >= 34 && pin <= 39) {
    pinMode(pin, INPUT);
  } else {
    pinMode(pin, INPUT_PULLUP);
  }
}

void IRAM_ATTR onLeftPulse() {
  leftPulseCount++;
}

void IRAM_ATTR onRightPulse() {
  rightPulseCount++;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupEncoderInputPin(ENCODER_LEFT_PIN);
  setupEncoderInputPin(ENCODER_RIGHT_PIN);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), onLeftPulse, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), onRightPulse, FALLING);

  Serial.println();
  Serial.println("=== ENCODER PULSE TEST START ===");
  Serial.print("LEFT_PIN=");
  Serial.print(ENCODER_LEFT_PIN);
  Serial.print(" RIGHT_PIN=");
  Serial.println(ENCODER_RIGHT_PIN);
  Serial.println("Log: revL revR revTotal | pulseL pulseR");
}

void loop() {
  if (millis() - lastLogMs >= 200) {
    noInterrupts();
    long pulseL = leftPulseCount;
    long pulseR = rightPulseCount;
    interrupts();

    float revL = (float)pulseL / (float)PULSES_PER_REV;
    float revR = (float)pulseR / (float)PULSES_PER_REV;
    float revTotal = revL + revR;

    Serial.print("Rev: ");
    Serial.print(revL, 3);
    Serial.print(" ");
    Serial.print(revR, 3);
    Serial.print(" ");
    Serial.print(revTotal, 3);
    Serial.print(" | pulse=");
    Serial.print(pulseL);
    Serial.print(" ");
    Serial.println(pulseR);

    lastLogMs = millis();
  }
}
