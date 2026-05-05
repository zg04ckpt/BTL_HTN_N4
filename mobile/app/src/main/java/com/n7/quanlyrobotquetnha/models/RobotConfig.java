package com.n7.quanlyrobotquetnha.models;

/**
 * Tham số runtime đồng bộ với firmware (module remote / NVS runtime_cfg).
 */
public class RobotConfig {
    public int basePwm;
    public float leftGain;
    public float rightGain;
    public float wallDistanceCm;
    public float endDistanceCm;
    public int minStartPwmL;
    public int minStartPwmR;

    public RobotConfig() {
        basePwm = 85;
        leftGain = 1.0f;
        rightGain = 1.0f;
        wallDistanceCm = 20.0f;
        endDistanceCm = 10.0f;
        minStartPwmL = 50;
        minStartPwmR = 50;
    }
}
