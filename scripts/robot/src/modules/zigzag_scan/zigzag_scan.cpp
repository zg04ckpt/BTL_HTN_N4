#include "zigzag_scan.h"

#include <Arduino.h>

#include "../../../config.h"
#include "../../hardwares/hardware.h"
#include "../moving/moving.h"
#include "../remote/remote.h"

// Kiểm tra siêu âm: có tường gần trong ngưỡng wallDistanceCm (runtime) thì cần quay đầu.
static bool checkWallDistanceToRotate() {
    const RuntimeConfig& cfg = getRuntimeConfig();
    static unsigned long lastCheckTime = 0;
    static float distance = 0;

    if (lastCheckTime == 0 || millis() - lastCheckTime > DIST_READ_INTERVAL_MS) {
        lastCheckTime = millis();
        distance = readDistanceCm();
    }

    Serial.print(F("[SCHEDULE] Khoảng cách đến tường: "));
    Serial.println(distance);
    if (distance >= 0.0f && distance < cfg.wallDistanceCm) {
        return true;
    }
    return false;
}

// Quay 180° một bánh (luân phiên trái/phải theo biến bên ngoài).
static RotateStatus handleRotate(bool isLeft) {
    return rotateByDeltaDegOneWheel(180.0f, isLeft);
}

void runZigzacBox() {
    Serial.println(F("[SCHEDULE] Chạy chế độ 1: Quét zigzag hộp chữ nhật"));

    bool nextRotateLeft = false;
    MoveStatus moveStatus = MoveStatus::FORWARD;

    while (true) {
        pollRemote();
        if (!remoteShouldRunScheduler()) {
            stop();
            return;
        }

        switch (moveStatus) {
            case MoveStatus::FORWARD:
                if (!checkWallDistanceToRotate()) {
                    goForwardWithNoPID();
                    Serial.println(F("[SCHEDULE] Đi thẳng"));
                    break;
                }

                moveStatus = MoveStatus::ROTATE;
                break;

            case MoveStatus::ROTATE:
                Serial.println(F("[SCHEDULE] Chuẩn bị quay đầu"));
                {
                    const RotateStatus result = handleRotate(nextRotateLeft);

                    Serial.print(F("[SCHEDULE] Kết thúc quay đầu"));

                    if (result == RotateStatus::SUCCESS) {
                        nextRotateLeft = !nextRotateLeft;
                        moveStatus = MoveStatus::FORWARD;
                    } else if (result == RotateStatus::TIMEOUT) {
                        Serial.println(F("[SCHEDULE] Quay đầu thất bại, không thực hiện được thao tác"));
                        stop();
                        return;
                    } else if (result == RotateStatus::WALL_DETECTED) {
                        Serial.println(F("[SCHEDULE] Quay đầu thất bại, gặp tường khi quay đầu, tạm thời kết thúc"));
                        stop();

                        // TODO: Xử lý quay về nhà

                        return;
                    }
                }
                break;
        }
        delay(LOOP_DELAY_MS);
    }
}
