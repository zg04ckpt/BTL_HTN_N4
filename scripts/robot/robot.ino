#include "config.h"
#include "src/hardwares/hardware.h"
#include "src/modules/moving/moving.h"
#include "src/hardwares/mpu6050.h"
#include "src/modules/schedule/schedule.h"
#include "src/modules/go_home/go_home.h"

void setup() {
    Serial.begin(115200);
    Serial.println(F("[MAIN] Bắt đầu cấu hình"));
    setupHardware();
    setupSchedule();
    Serial.println(F("[MAIN] Cấu hình hoàn tất"));
}

#pragma region Temp 

#include "src/hardwares/encoder.h"
void debugPulseBalance() {
    static long prevL = 0;
    static long prevR = 0;
    static bool primed = false;
    static unsigned long lastMs = 0;
  
    // Giảm tần suất in để không nghẽn Serial
    if (millis() - lastMs < 100) return;
    lastMs = millis();
  
    long l = 0, r = 0;
    noInterrupts();
    l = leftPulses;
    r = rightPulses;
    interrupts();
  
    if (!primed) {
      prevL = l;
      prevR = r;
      primed = true;
      return;
    }
  
    long dL = l - prevL;   // xung bánh trái trong 100ms
    long dR = r - prevR;   // xung bánh phải trong 100ms
    long diff = dL - dR;   // >0: trái nhanh hơn, <0: phải nhanh hơn
  
    prevL = l;
    prevR = r;
  
    Serial.print("[ENC_BAL] dL=");
    Serial.print(dL);
    Serial.print(" dR=");
    Serial.print(dR);
    Serial.print(" diff=");
    Serial.println(diff);
}


void debugMPU6050() {
    Serial.print("[MPU6050] Yaw: ");
    Serial.println(readYawDeg());
}

  #pragma endregion

void loop() {
    // updateYawDeg();
    // debugMPU6050();
    // goForwardWithNoPID();
    // debugPulseBalance();
    // Test 4 lần mỗi vòng:
    // 1) +90 (bánh trái), 2) -90 (bánh trái), 3) -90 (bánh phải), 4) +90 (bánh phải)
    // rotateByDeltaDegOneWheel(90.0f, true);
    // delay(700);
    // rotateByDeltaDegOneWheel(-90.0f, true);
    // delay(700);
    // rotateByDeltaDegOneWheel(-90.0f, false);
    // delay(700);
    // rotateByDeltaDegOneWheel(90.0f, false);
    // delay(1400);

    // if (homeSignalPresent()) {
    //   Serial.println(F("[MAIN] Da bat duoc tin hieu home"));
    // } else {
    //   Serial.println(F("[MAIN] Khong"));
    // }
    run(); 

    delay(LOOP_DELAY_MS);
}