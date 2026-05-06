#include "ir_receiver.h"

#include "pin.h"
#include <Arduino.h>

constexpr int IR_RECV_PIN = 19;

namespace {

// Quét ngắn: đếm mẫu LOW — IR điều chế thường kéo xuống LOW khi có sóng mang sau module TSOP.
constexpr unsigned long kSampleWindowMs = 25;
constexpr int kMinLowSamples = 4;
constexpr unsigned kPollIntervalUs = 120;

}  // namespace

void setupIRReceiver() {
    pinMode(IR_RECV_PIN, INPUT_PULLUP);
}

bool irSignalPresent() {
    const unsigned long start = millis();
    int lowSamples = 0;
    int totalSamples = 0;

    while (millis() - start < kSampleWindowMs) {
        totalSamples++;
        if (digitalRead(IR_RECV_PIN) == LOW) {
            lowSamples++;
        }
        delayMicroseconds(kPollIntervalUs);
    }

    return lowSamples >= kMinLowSamples && totalSamples > 0;
}
