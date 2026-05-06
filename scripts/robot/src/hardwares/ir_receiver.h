#pragma once

#include <Arduino.h>

void setupIRReceiver();

/// Trả về true nếu trong cửa sổ lấy mẫu có hoạt động IR (TSOP / module giải điều chế — mức LOW khi có burst 38 kHz).
bool irSignalPresent();
