#pragma once
#include <stdint.h>
#include <Arduino.h>

enum class RotateStatus {
    SUCCESS,
    TIMEOUT,
    WALL_DETECTED,
};

// Setup tất cả cấu hình cần thiết cho module Moving
void setupMoving();

// Đi thẳng dùng PID
void goForward(bool isBackward = false);

// Đi thẳng bình thường, không dùng PID
void goForwardWithNoPID(bool isBackward = false);

// Dừng motor theo cơ chế phanh chuẩn giống file final.
void stop();


// Quay theo góc tương đối nhưng chỉ dùng 1 bánh:
// - useLeftWheel = true: chỉ bánh trái chạy, bánh phải giữ 0
// - useLeftWheel = false: chỉ bánh phải chạy, bánh trái giữ 0
// Trả về true nếu quay thành công, false nếu quay thất bại (khi gặp chướng ngại vậy trong lúc quay)
RotateStatus rotateByDeltaDegOneWheel(float deltaDeg, bool isLeftRotate);