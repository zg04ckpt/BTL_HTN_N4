#include <Arduino.h>

// Line sensor test on SVN pin of ESP32 (GPIO39)
// GPIO39 is input-only and supports analog read.
const int LINE_PIN = 39; // SENSOR_VN / SVN

// Tune this threshold after watching raw values.
int lineThreshold = 2000;

unsigned long lastLogMs = 0;

int readLineRawAverage(int samples = 10) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(LINE_PIN);
    delay(2);
  }
  return (int)(sum / samples);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LINE_PIN, INPUT);
  analogReadResolution(12); // ESP32 default range: 0..4095

  Serial.println();
  Serial.println("=== LINE SENSOR TEST START ===");
  Serial.println("LINE=GPIO39 (SVN)");
  Serial.println("Put sensor on white and black to tune threshold");
}

void loop() {
  if (millis() - lastLogMs >= 200) {
    int raw = readLineRawAverage(10);

    // Common behavior: white reflects more, black reflects less.
    // Depending on module, this can be inverted.
    bool isBlack = raw < lineThreshold;

    Serial.print("Raw=");
    Serial.print(raw);
    Serial.print(" | Threshold=");
    Serial.print(lineThreshold);
    Serial.print(" | Detected=");
    Serial.println(isBlack ? "BLACK" : "WHITE");

    lastLogMs = millis();
  }
}
