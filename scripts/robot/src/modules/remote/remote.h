#pragma once

// Cấu hình runtime (điều chỉnh từ app, lưu NVS) — thuộc module remote.
struct RuntimeConfig {
    int basePwm;
    float leftGain;
    float rightGain;
    float wallDistanceCm;
    float endDistanceCm;
    int minStartPwmL;
    int minStartPwmR;
};

// Đọc cấu hình đã lưu từ NVS — gọi một lần trong setup() (vd. robot.ino) trước khi chạy scheduler/moving.
void setupRuntimeConfig();
const RuntimeConfig& getRuntimeConfig();
void setRuntimeConfig(const RuntimeConfig& cfg);
void saveRuntimeConfig();

// Khởi tạo module điều khiển từ xa (Wi‑Fi + WebSocket đường dẫn /ws).
void setupRemote();

// Xử lý việc remote theo chu kỳ loop (không chặn luồng).
void pollRemote();

// Robot có được phép chạy scheduler hay không (bật/tắt từ app).
bool remoteShouldRunScheduler();
