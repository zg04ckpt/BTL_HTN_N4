#pragma once

#include <stdint.h>

// Trạng thái một bước trong vòng quét zigzag (đi thẳng / quay).
enum class MoveStatus : uint8_t {
    FORWARD,
    ROTATE,
};

void runZigzacBox();
