#define IR_PIN 13
#define IR_FREQ 38000
#define IR_RES 8

void setup() {
  Serial.begin(115200);

  bool success = ledcAttach(IR_PIN, IR_FREQ, IR_RES);
  
  if (success) {
    ledcWrite(IR_PIN, 128); 
    Serial.println(">>> TRU PHAT DANG BAN SONG 38KHZ (Version 3.x)!");
  } else {
    Serial.println(">>> LOI: Khong the thiet lap LEDC!");
  }
}

void loop() {
}