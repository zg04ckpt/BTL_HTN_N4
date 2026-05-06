#pragma once

#include <Arduino.h>

// Setup tất cả cấu hình cần thiết cho cảm biến vạch (TCRT5000)
void setupLineSensor();

// Đọc giá trị analog thô từ cảm biến (ESP32: 0..4095)
int readLineSensorRaw();

// Trả về true nếu đang gặp vạch đen (theo ngưỡng cấu hình)
bool isBlackLineDetected();

