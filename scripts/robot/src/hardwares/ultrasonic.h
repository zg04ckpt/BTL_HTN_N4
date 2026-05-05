#pragma once

#include <Arduino.h>

// Setup tất cả cấu hình cần thiết cho Ultrasonic
void setupUltrasonic();

// Đọc khoảng cách phía trước (cm)
float readDistanceCm();
