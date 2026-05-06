#pragma once

/// Điều kiện về nhà: pin, vùng làm việc, lệnh remote, ... — TODO gắn cảm biến / NVS / remote.
bool shouldGoHome();

/// Quay tại chỗ theo từng bước để tìm tín hiệu home/dock.
/// Khi phát hiện tín hiệu → `stop()`, kết thúc hàm và trả về true.
/// Hết thời gian / quét đủ một vòng mà không thấy → `stop()`, return false.
bool searchHomeSignalRotateInPlace();

/// Đi thẳng về home; nếu mất tín hiệu (lệch / khuất) → dừng và gọi lại `searchHomeSignalRotateInPlace()`, rồi tiếp tục.
/// TODO: thoát khi `isDockedAtHome()` hoặc timeout toàn tuyến.
void navigateStraightTowardHomeWithRecovery();

/// Trả về true nếu có tín hiệu home/dock.
bool homeSignalPresent();