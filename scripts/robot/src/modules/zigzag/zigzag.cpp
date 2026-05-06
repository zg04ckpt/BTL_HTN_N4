#include "zigzag.h"

#include <Arduino.h>

#include "../../../config.h"
#include "../../hardwares/hardware.h"
#include "../moving/moving.h"
#include "../schedule/schedule.h"

#pragma region Chế độ 1: Quét zigzac hộp chữ nhật

namespace {

// Kiểm tra xem có gặp tường gần không để quay đầu
bool checkWallDistanceToRotate() {
    static unsigned long lastCheckTime = 0;
    static float distance = 0;

    if (lastCheckTime == 0 || millis() - lastCheckTime > DIST_READ_INTERVAL_MS) {
        lastCheckTime = millis();
        distance = readDistanceCm();
    }

    Serial.print(F("[ZIGZAG] Khoảng cách đến tường: "));
    Serial.println(distance);
    if (distance >= 0.0f && distance < WALL_DISTANCE_CM) {
        return true;
    }
    return false;
}

RotateStatus handleRotate(bool isLeft) {
    return rotateByDeltaDegOneWheel(180.0f, isLeft);
}

}  // namespace

void runZigzacBox() {
    Serial.println(F("[ZIGZAG] Chạy chế độ 1: Quét zigzac hộp chữ nhật"));

    bool nextRotateLeft = false;
    MoveStatus moveStatus = MoveStatus::FORWARD;

    while (true) {
        switch (moveStatus) {
            case MoveStatus::FORWARD:
                if (!checkWallDistanceToRotate()) {
                    goForwardWithNoPID();
                    Serial.println(F("[ZIGZAG] Đi thẳng"));
                    break;
                }
                moveStatus = MoveStatus::ROTATE;
                break;

            case MoveStatus::ROTATE: {
                Serial.println(F("[ZIGZAG] Chuẩn bị quay đầu"));
                const RotateStatus result = handleRotate(nextRotateLeft);

                Serial.println(F("[ZIGZAG] Kết thúc quay đầu"));

                if (result == RotateStatus::SUCCESS) {
                    nextRotateLeft = !nextRotateLeft;
                    moveStatus = MoveStatus::FORWARD;
                } else if (result == RotateStatus::TIMEOUT) {
                    Serial.println(F("[ZIGZAG] Quay đầu thất bại, không thực hiện được thao tác"));
                    stop();
                    return;
                } else if (result == RotateStatus::WALL_DETECTED) {
                    Serial.println(F("[ZIGZAG] Quay đầu thất bại, gặp tường khi quay đầu, tạm thời kết thúc"));
                    stop();

                    // TODO: gọi hàm goHome()
                    goHome();

                    return;
                }
                break;
            }
            default:
                break;
        }
        delay(LOOP_DELAY_MS);
    }
}

#pragma endregion
