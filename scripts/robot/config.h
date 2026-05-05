constexpr int LOOP_DELAY_MS = 80; // Độ trễ mỗi vòng khi đi thẳng (ổn định vòng lặp)
constexpr int ERROR_TIME_OUT = 80000; // Thời gian tối đa cho một thao tác (dự phòng)
// constexpr float WHEEL_RADIUS_CM = 3.4f; // Bán kính bánh xe (cm)
// constexpr int ENCODER_SLOTS_PER_WHEEL_REV = 20; // Số xung trên 1 vòng quay

constexpr float LEFT_GAIN = 2.0f; // Hệ số chỉnh PWM bánh trái khi đi thẳng
constexpr float RIGHT_GAIN = 1.0f; // Hệ số chỉnh PWM bánh phải khi đi thẳng
constexpr int BASE_PWM = 85; // PWM cơ bản cho mỗi bánh khi đi thẳng.

constexpr float WALL_DISTANCE_CM = 22.0f; // Gặp vật cản trước mặt ở khoảng này thì coi là "gặp tường"
constexpr float END_DISTANCE_CM = 10.0f; // Ngưỡng khoảng cách kết thúc vùng làm việc / vật cản rất gần (cm)
// constexpr float WALL_CLEAR_CM = 30.0f; // Lùi xa hơn ngưỡng này mới cho phép lại phát hiện tường
// constexpr float HOME_BACKUP_MAX_CM = 20.0f; // Lùi tối đa khi gặp chướng ngại lúc quay (theo yêu cầu lịch trình)
constexpr int MIN_START_PWM_L = 50; // PWM tối thiểu để bánh trái bắt đầu quay (bù ma sát bánh trái)
constexpr int MIN_START_PWM_R = 50; // PWM tối thiểu để bánh phải bắt đầu quay
constexpr unsigned long DIST_READ_INTERVAL_MS = 80; // Khoảng thời gian đọc khoảng cách (ms)
// constexpr float DECLINATION_DEG = 0.0f; // Độ lệch từ trường tự nhiên của Trái Đất (độ)