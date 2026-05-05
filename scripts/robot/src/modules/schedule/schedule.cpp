#include "schedule.h"
#include "../../hardwares/hardware.h"
#include "../../modules/moving/moving.h"
#include <Arduino.h>
#include "../../../config.h"

// Biến toàn cục
RobotStatus g_robotStatus;
RunMode g_runMode;

// Cấu hình
void setupSchedule() {
    Serial.println(F("[SCHEDULE] Cấu hình lịch làm việc"));
    g_robotStatus = RobotStatus::SLEEPING;
    g_runMode = RunMode::ZIGZAC_BOX;
}

// Hàm chạy 
void run() {
    switch (g_robotStatus) {
        case RobotStatus::SLEEPING:
            sleep();
            break;
        case RobotStatus::WORKING:
            work();
            break;
        case RobotStatus::GOING_HOME:
            goHome();
            break;
        case RobotStatus::ERROR:
            error();
            break;
        
    }
}

// Hàm xử lý trạng thái ngủ
void sleep() {
    Serial.println(F("[SCHEDULE] Đang ngủ"));
    g_robotStatus = RobotStatus::SLEEPING;

    // temp
    g_robotStatus = RobotStatus::WORKING;
}

// Hàm xử lý trạng thái đi về nhà
void goHome() {
    Serial.println(F("[SCHEDULE] Đi về nhà"));
    g_robotStatus = RobotStatus::GOING_HOME;
}

// Hàm xử lý trạng thái lỗi
void error() {
    Serial.println(F("[SCHEDULE] Lỗi"));
    g_robotStatus = RobotStatus::ERROR;
}





#pragma region Chế độ 1: Quét zigzac hộp chữ nhật

// Kiểm tra xem có gặp tường gần không để quay đầu
bool checkWallDistanceToRotate() {
    static unsigned long lastCheckTime = 0;
    static float distance = 0;

    if (lastCheckTime == 0 || millis() - lastCheckTime > DIST_READ_INTERVAL_MS) {
        lastCheckTime = millis();
        distance = readDistanceCm();
    }

    Serial.print(F("[SCHEDULE] Khoảng cách đến tường: "));
    Serial.println(distance);
    if (distance < WALL_DISTANCE_CM) {
        return true;
    }
    return false;
}

// Xử lý quay đầu
RotateStatus handleRotate(bool isLeft) {

    RotateStatus result;
    result = rotateByDeltaDegOneWheel(180.0f, isLeft);
    return result;
}

// Quét zigzac hộp chữ nhật
void runZigzacBox() {
    Serial.println(F("[SCHEDULE] Chạy chế độ 1: Quét zigzac hộp chữ nhật"));

    bool nextRotateLeft = false; // Biến kiểm soát hướng quay đầu
    MoveStatus moveStatus = MoveStatus::FORWARD; // Biến kiểm soát hướng di chuyển

    while (true) {
        switch (moveStatus) {
            case MoveStatus::FORWARD:
                // Nếu chưa gặp tường thì tiếp tục đi thẳng
                if (!checkWallDistanceToRotate()) {
                    goForwardWithNoPID();
                    Serial.println(F("[SCHEDULE] Đi thẳng"));
                    break;
                }

                // Nếu gặp tường thì quay đầu
                moveStatus = MoveStatus::ROTATE;
                break;
            case MoveStatus::ROTATE:
                Serial.println(F("[SCHEDULE] Chuẩn bị quay đầu"));
                RotateStatus result = handleRotate(nextRotateLeft);

                Serial.print(F("[SCHEDULE] Kết thúc quay đầu"));

                // Xử lý kết quả quay đầu
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
                break;

        }
        delay(LOOP_DELAY_MS);
    }
}



#pragma endregion

// Hàm xử lý trạng thái làm việc
void work() {
    Serial.println(F("[SCHEDULE] Làm việc"));
    g_robotStatus = RobotStatus::WORKING;

    switch (g_runMode) {
        case RunMode::ZIGZAC_BOX:
            runZigzacBox();
            break;
    }
}