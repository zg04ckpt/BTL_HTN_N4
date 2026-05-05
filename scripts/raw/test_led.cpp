#define IR_PIN 13       // Chân nối vào trở 1000 Ohm -> Base 2N2222
#define IR_FREQ 38000   // Tần số 38kHz chuẩn SM0038
#define IR_RES 8        // Độ phân giải 8 bit (từ 1 đến 15)

void setup() {
  Serial.begin(115200);

  // Ở phiên bản mới, bạn chỉ cần 1 dòng này thay cho ledcSetup và ledcAttachPin
  // Cú pháp: ledcAttach(pin, freq, resolution);
  bool success = ledcAttach(IR_PIN, IR_FREQ, IR_RES);
  
  if (success) {
    // Phát xung với Duty Cycle 50% (với 8 bit thì 128 là một nửa của 255)
    ledcWrite(IR_PIN, 128); 
    Serial.println(">>> TRU PHAT DANG BAN SONG 38KHZ (Version 3.x)!");
  } else {
    Serial.println(">>> LOI: Khong the thiet lap LEDC!");
  }
}

void loop() {
  // PWM phần cứng tự chạy ngầm
}