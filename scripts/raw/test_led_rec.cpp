const int IR_RECEIVER_PIN = 18; // Chân nối với OUT của SM0038

void setup() {
  Serial.begin(115200);
  pinMode(IR_RECEIVER_PIN, INPUT);
  Serial.println("Dang doi tin hieu tu tru phat...");
}

void loop() {
  int sensorValue = digitalRead(IR_RECEIVER_PIN);

  if (sensorValue == LOW) {
    Serial.println(">>> DA NHAN DUOC TIN HIEU 38KHZ!");
  } else {
    // Không in gì để tránh rác Serial, hoặc in dấu chấm
    // Serial.print(".");
  }
  delay(100); 
}