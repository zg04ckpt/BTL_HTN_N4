#pragma once

// Setup tất cả cấu hình cần thiết cho Encoder
void setupEncoder();

// Số xung Encoder bánh trái
extern volatile long leftPulses;

// Số xung Encoder bánh phải
extern volatile long rightPulses;