#include "hardware.h"
#include "encoder.h"
#include "ultrasonic.h"
#include "l298n.h"
#include "mpu6050.h"
#include "line_sensor.h"

// Setup tất cả cấu hình cần thiết cho hardware
void setupHardware() {
    setupEncoder();
    setupUltrasonic();
    setupL298N();
    setupMPU6050();
    setupIRReceiver();
    setupLineSensor();
}