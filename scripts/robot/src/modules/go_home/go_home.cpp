#include "go_home.h"

#include "../../../config.h"
#include "../../hardwares/ir_receiver.h"
#include "../../hardwares/line_sensor.h"
#include "../moving/moving.h"
#include <Arduino.h>

namespace {


bool dockArrivedOrDocked() {
    // Điều kiện về dock: gặp vạch đen từ TCRT5000 (ưu tiên trong quá trình đi về nhà).
    // Có thể mở rộng thêm: công tắc đế, đủ gần siêu âm, v.v.
    return isBlackLineDetected();
}

constexpr float kSearchRotateStepDeg = 15.0f;
constexpr unsigned long kSearchRotateTimeoutMs = 120000;
constexpr unsigned long kNavigateOverallTimeoutMs = 300000;

}  // namespace

bool homeSignalPresent() {
    return irSignalPresent();
}


bool shouldGoHome() {
    // TODO: pin yếu, hết khu vực lau (END_DISTANCE / zone), remote "go_home", cờ schedule...
    return false;
}

bool searchHomeSignalRotateInPlace() {
    const unsigned long t0 = millis();
    float turned = 0.0f;

    while (millis() - t0 < kSearchRotateTimeoutMs) {
        if (homeSignalPresent()) {
            stop();
            Serial.println(F("[GO_HOME] Da bat duoc tin hieu home — dung tai cho."));
            return true;
        }

        const RotateStatus st = rotateByDeltaDegOneWheel(kSearchRotateStepDeg, true);
        if (st != RotateStatus::SUCCESS) {
            stop();
            Serial.println(F("[GO_HOME] Quay tim tin hieu that bai (timeout/vat can)."));
            return false;
        }

        turned += kSearchRotateStepDeg;
        if (turned >= 360.0f) {
            turned = 0.0f;
            // TODO: có thể đổi chiều quay hoặc bước nhỏ hơn sau mỗi vòng
        }
    }

    stop();
    Serial.println(F("[GO_HOME] Het thoi gian tim tin hieu."));
    return false;
}

void navigateStraightTowardHomeWithRecovery() {
    Serial.println(F("[GO_HOME] Bat dau ve home (thang + tim lai tin hieu neu mat)."));

    const unsigned long navStart = millis();

    while (!dockArrivedOrDocked()) {
        if (millis() - navStart >= kNavigateOverallTimeoutMs) {
            stop();
            Serial.println(F("[GO_HOME] Het thoi gian navigation — TODO dieu chinh / ERROR."));
            return;
        }

        if (isBlackLineDetected()) {
            stop();
            Serial.println(F("[GO_HOME] Da gap vach den — coi nhu da ve dock."));
            return;
        }

        if (!homeSignalPresent()) {
            stop();
            Serial.println(F("[GO_HOME] Mat tin hieu — quay tai cho tim lai."));
            if (!searchHomeSignalRotateInPlace()) {
                Serial.println(F("[GO_HOME] Chua tim lai duoc tin hieu — TODO chinh sach retry / canh bao."));
                delay(500);
            }
            continue;
        }

        goForwardWithNoPID();
        delay(LOOP_DELAY_MS);
    }

    stop();
    Serial.println(F("[GO_HOME] Da ve dock / home (placeholder)."));
}
