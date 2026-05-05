#pragma once

// Góc yaw hiện tại (độ), tích phân từ gyro Z
extern float yawDeg;

// Khởi tạo MPU6050
void setupMPU6050();

// Đọc góc yaw hiện tại
float readYawDeg();

// Cập nhật yaw từ gyro Z (gọi mỗi vòng loop)
void updateYawDeg();

