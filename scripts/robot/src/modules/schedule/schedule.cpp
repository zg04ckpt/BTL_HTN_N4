#include "schedule.h"
#include "../zigzag_scan/zigzag_scan.h"
#include <Arduino.h>

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

    // Tạm thời: bỏ chờ ngủ, chuyển thẳng sang làm việc (thử nghiệm).
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
