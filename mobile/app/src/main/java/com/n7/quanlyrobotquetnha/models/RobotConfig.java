package com.n7.quanlyrobotquetnha.models;

public class RobotConfig {
    public String ssid;
    public String pass;
    public int goSpeed;
    public int turnSpeed;
    public float wheelRadiusCm;
    public float rotateReverseThresholdDeg;
    public float rotateToleranceDeg;
    public int rotateMaxCorrections;
    public float wallStopDistanceCm;
    public int sleepMinutes;
    public boolean pulseEnabled;
    public float pulseFreqHz;
    public float pulseDutyPercent;
    public float pulsePowerPercent;
    
    // Zigzag navigation parameters
    public float zigzagAngleOffsetM;        // M: Phần bù góc khi điều chỉnh hướng
    public float zigzagDeviationThresholdN; // N: Ngưỡng lệch góc cho phép
    public float zigzagStraightDistanceK;   // K: Quãng đường đi thẳng tối đa (cm)
    public float zigzagOffsetDistanceL;     // L: Độ dịch ngang khi chuyển hàng

    public RobotConfig() {
        // Default values
        ssid = "";
        pass = "";
        goSpeed = 60;
        turnSpeed = 60;
        wheelRadiusCm = 3.4f;
        rotateReverseThresholdDeg = 8.0f;
        rotateToleranceDeg = 2.0f;
        rotateMaxCorrections = 2;
        wallStopDistanceCm = 20.0f;
        sleepMinutes = 10;
        pulseEnabled = true;
        pulseFreqHz = 12.5f;
        pulseDutyPercent = 50.0f;
        pulsePowerPercent = 100.0f;
        zigzagAngleOffsetM = 28.0f;
        zigzagDeviationThresholdN = 5.0f;
        zigzagStraightDistanceK = 100.0f;
        zigzagOffsetDistanceL = 30.0f;
    }
}
