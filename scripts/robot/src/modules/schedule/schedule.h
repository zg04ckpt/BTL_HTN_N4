#pragma once
#include <stdint.h>

enum class RunMode : uint8_t {
    ZIGZAC_BOX, // Quét zigzag trong hộp chữ nhật
};

enum class RobotStatus : uint8_t {
    SLEEPING, // Robot đang chờ / ngủ trước khi bắt đầu chạy
    WORKING, // Robot đang chạy
    GOING_HOME, // Robot đang đi về nhà
    ERROR, // Robot đang bị lỗi
    // STOPPED, // Robot đang dừng
};

// Biến toàn cục
extern RobotStatus g_robotStatus;
extern RunMode g_runMode;

// Cấu hình
void setupSchedule();

// Chạy
void run();

// Các hàm điều khiển
void work();
void sleep();
void goHome();
void error();
void stop();
