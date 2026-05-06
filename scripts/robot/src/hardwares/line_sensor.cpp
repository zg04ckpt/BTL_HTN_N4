#include "line_sensor.h"

#include "../../config.h"
#include "pin.h"

void setupLineSensor() {
    pinMode(LINE_SENSOR_PIN, INPUT);
}

int readLineSensorRaw() {
    return analogRead(LINE_SENSOR_PIN);
}

bool isBlackLineDetected() {
    const int v = readLineSensorRaw();
    if (LINE_SENSOR_BLACK_IS_LOW) {
        return v <= LINE_BLACK_THRESHOLD;
    }
    return v >= LINE_BLACK_THRESHOLD;
}

