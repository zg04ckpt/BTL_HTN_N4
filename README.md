# Dự án phát triển robot lau nhà - Nhóm 4
Dự án 

## Các thành phần chính
- Robot lau nhà 
- Nhà robot (Nơi robot sleep)
- Ứng dụng quản lý

## Các module phát triển firmware

### 1. Module di chuyển
- Bao gồm việc tối ưu 2 hành động move chính của robot là xoay bằng 1 bánh + đi thẳng
- Source path: scripts\robot\src\modules\moving

### 2. Module zigzag scan
- Phát triển cơ chế di chuyển đơn giản nhất, dùng 2 thao tác xoay và đi thẳng để điều khiển robot đi hình zigzag để đảm bảo đạt được độ phủ tối đa trên 1 khu vực trống.
- Source path: scripts\robot\src\modules\zigzag_scan

### 3. Module nhà
- Phát triển cơ chế dò tìm phương hướng và di chuyển về nhà sau khi robot hoàn thành công việc quét dọn
- Source path: scripts\robot\src\modules\home

### 4. Module điều khiển từ xa
- Phát triển cơ chế kết nối giữa robot với app mobile quản lý qua mạng nội bộ, giúp user có thể cấu hình các thông số từ xa hoặc ra lệch cưỡng chế đối với robot.
- Source path: scripts\robot\src\modules\remote

### 5. Module lịch trình
- Đây là cơ chế cốt lõi của robot, nó điều khiển robot thực hiện tuần tự các tác vụ, lên lịch cho hành động tiếp theo, đảm bảo robot có thể hoạt động độc lập theo chu kì sleep <-> work.
- Source path: scripts\robot\src\modules\schedule