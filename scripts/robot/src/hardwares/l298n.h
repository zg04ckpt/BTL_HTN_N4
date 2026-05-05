#pragma once

#include <Arduino.h>

// Setup tất cả cấu hình cần thiết cho L298N
void setupL298N();

// Điều khiển trực tiếp 2 bánh
// - pwmL/pwmR: 0..255
// - forwardL/forwardR: true = tiến, false = lùi
void setMotor(int pwmL, bool forwardL, int pwmR, bool forwardR);

// Dừng motor kiểu "chuẩn final":
// 1) phanh gấp, 2) giữ phanh ngắn, 3) ngắt điện hoàn toàn.
void stopMotor();