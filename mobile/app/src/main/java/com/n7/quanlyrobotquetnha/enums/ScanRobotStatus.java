package com.n7.quanlyrobotquetnha.enums;

/**
 * Trạng thái quét robot
 */
public enum ScanRobotStatus {
    /**
     * Đang tìm robot
     */
    SCANNING,
    /**
     * Robot chưa bật
     */
    NOT_FOUND,

    /**
     * Robot chưa được cấu hình wifi / ko kết nối được wifi, đang bật AP để nhận cấu hình wifi
     */
    NOT_CONFIG,

    /**
     * Robot đã kết nối wifi khác (hiển thị tên wifi đã lưu trong lịch sử + yêu cầu user chuyển sang mạng đó để kết nối)
     */
    CONNECTED_IN_OTHER_WIFI,

    /**
     * Robot đang cùng mạng và sẵn sàng kết nối
     */
    CAN_CONNECT,

    /**
     * Có lỗi xảy ra
     */
    ERROR
}
