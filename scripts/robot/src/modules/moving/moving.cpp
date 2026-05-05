#include "moving.h"
#include "../../../config.h"
#include "../../hardwares/hardware.h"
#include "../../hardwares/encoder.h"
#include "../remote/remote.h"

// Biến
// PID cân bằng encoder
static float kpBal = 0.12f;
static float kiBal = 0.002f;
static float kdBal = 0.04f;
static float iBal = 0.0f;
static float prevBalErr = 0.0f;

// Setup tất cả cấu hình cần thiết cho module Moving
void setupMoving() {
    iBal = 0.0f;
    prevBalErr = 0.0f;
}

// #pragma region PID cân bằng encoder 

// float clampFloat(float v, float lo, float hi) {
//     if (v < lo) return lo;
//     if (v > hi) return hi;
//     return v;
// }

// float normalize180(float a) {
//     while (a > 180.0f) a -= 360.0f;
//     while (a < -180.0f) a += 360.0f;
//     return a;
// }

// static int computeBalanceCorrection(float dt) {
//     long l = 0, r = 0;
//     noInterrupts();
//     l = leftPulses;
//     r = rightPulses;
//     interrupts();

//     float err = (float)(l - r);
//     iBal += err * dt;
//     iBal = clampFloat(iBal, -3000.0f, 3000.0f);

//     float d = (err - prevBalErr) / dt;
//     prevBalErr = err;

//     float out = kpBal * err + kiBal * iBal + kdBal * d;
//     out = clampFloat(out, -50.0f, 50.0f);
//     return (int)out;
// }

// #pragma endregion

// Đi thẳng
// void goForward(bool isBackward) {
//     static unsigned long prevPidMs = millis();
//     unsigned long now = millis();
//     float dt = (now - prevPidMs) / 1000.0f;
//     prevPidMs = now;
//     if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;
//     int corr = computeBalanceCorrection(dt);
//     int pwmL = BASE_PWM - corr;
//     int pwmR = BASE_PWM + corr;
//     pwmL = constrain(pwmL, 0, 255);
//     pwmR = constrain(pwmR, 0, 255);
//     if (pwmL > 0 && pwmL < MIN_START_PWM_L) pwmL = MIN_START_PWM_L;
//     if (pwmR > 0 && pwmR < MIN_START_PWM_R) pwmR = MIN_START_PWM_R;
//     // int outL = constrain((int)(pwmL * LEFT_GAIN), 0, 255);
//     // int outR = constrain((int)(pwmR * RIGHT_GAIN), 0, 255);
//     const bool forward = !isBackward;
//     setMotor(pwmL, forward, pwmR, forward);
// }

void goForwardWithNoPID(bool isBackward) {
    const RuntimeConfig& cfg = getRuntimeConfig();
    const bool forward = !isBackward;
    int pwmL = (int)(cfg.basePwm * cfg.leftGain);
    int pwmR = (int)(cfg.basePwm * cfg.rightGain);
    pwmL = constrain(pwmL, 0, 255);
    pwmR = constrain(pwmR, 0, 255);
    if (pwmL > 0 && pwmL < cfg.minStartPwmL) pwmL = cfg.minStartPwmL;
    if (pwmR > 0 && pwmR < cfg.minStartPwmR) pwmR = cfg.minStartPwmR;
    setMotor(pwmL, forward, pwmR, forward);
}

void stop() {
    stopMotor();
}

// Kiểm tra xem có gặp tường gần khi quay đầu không
bool checkWallDistanceWhenRotating() {
    const RuntimeConfig& cfg = getRuntimeConfig();
    float distance = readDistanceCm();
    if (distance >= 0.0f && distance < cfg.endDistanceCm) {
        return true;
    }
    return false;
}

RotateStatus rotateByDeltaDegOneWheel(float deltaDeg, bool isLeftRotate) {

    updateYawDeg();
    const float startYaw = readYawDeg();
    float targetYaw = startYaw;
    if (isLeftRotate) {
        targetYaw += deltaDeg;
    } else {
        targetYaw -= deltaDeg;
    }

    const unsigned long startTimeMs = millis();
    unsigned long lastLogMs = startTimeMs;

    while (true) {
        pollRemote();

        // Hết time quay
        if (millis() - startTimeMs > ERROR_TIME_OUT) {
            stop();
            Serial.println(F("[MOVE] rotateByDeltaDeg TIMEOUT"));
            return RotateStatus::TIMEOUT;
        }

        // Kiểm tra xem có gặp tường gần khi quay đầu không
        // if (checkWallDistanceWhenRotating()) {
        //     stop();
        //     Serial.println(F("[MOVE] rotateByDeltaDeg gặp tường khi quay đầu"));
        //     return RotateStatus::WALL_DETECTED;
        // }

        updateYawDeg();
        const float curYaw = readYawDeg();
        const float err = targetYaw - curYaw;

        // Log
        if (millis() - lastLogMs >= 100) {
            Serial.print(F("[MOVE][MAIN] target="));
            Serial.print(targetYaw, 2);
            Serial.print(F(" cur="));
            Serial.print(curYaw, 2);
            Serial.print(F(" err="));
            Serial.println(err, 2);
            lastLogMs = millis();
        }

        // Sai số nhỏ hơn 1 độ dừng luôn
        if (fabsf(err) <= 1.0f) {
            stop();
            Serial.println(F("[MOVE] rotateByDeltaDeg đạt góc"));
            return RotateStatus::SUCCESS;
        }

        // Tính lực quay cho phù hợp
        int pwm = 60;
        if (fabsf(err) >= 60.0f) {
            pwm = 180;
        } else if (fabsf(err) >= 40.0f) {
            pwm = 120;
        } else if (fabsf(err) >= 30.0f) {
            pwm = 90;
        }
        // pwm = pwm * 0.7f;

        // Nếu là cần quay phải thì chỉ quay bánh trái, giữ bánh phải
        if (!isLeftRotate) {

            // Nếu err > 0 tức là chưa đạt đủ góc quay => bánh trái phải quay tiến
            if (err > 0.0f) {
                setMotor(pwm, false, 0, true); // Bánh trái tiển, bánh phải giữ 0
            
            // Nếu err < 0 tức là đã quay quá góc quay => bánh trái phải quay lùi
            } else {
                setMotor(pwm, true, 0, true); // Bánh trái lùi, bánh phải giữ 0
            }
        }

        // Logic tương tự với quay trái
        if (isLeftRotate) {

            // Nếu err > 0 tức là chưa đạt đủ góc quay => bánh phải phải quay tiến
            if (err > 0.0f) {
                setMotor(0, true, pwm * 0.5f, true); // Bánh phải tiển, bánh trái giữ 0
            
            // Nếu err < 0 tức là đã quay quá góc quay => bánh phải phải quay lùi
            } else {
                setMotor(0, true, pwm * 0.5f, false);  // Bánh phải lùi, bánh trái giữ 0
            }
        }

        delay(20);
    }

    return RotateStatus::TIMEOUT;
}