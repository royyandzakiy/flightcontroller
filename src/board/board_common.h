// Helpers shared by the board implementations. Pure arithmetic, no hardware access.
#pragma once

#include <stdint.h>

namespace board {

// OneShot125: 125 us = stop, 250 us = full throttle.
static const float ONESHOT_MIN_US = 125.0f;
static const float ONESHOT_MAX_US = 250.0f;

inline float oneshotPulseUs(float command)
{
    if (!(command > 0.0f)) command = 0.0f;  // also catches NaN
    if (command > 1.0f) command = 1.0f;
    return ONESHOT_MIN_US + command * (ONESHOT_MAX_US - ONESHOT_MIN_US);
}

// SOS in Morse, one entry per 150 ms step: 1 = LED on.
static const uint8_t SOS_PATTERN[] = {
    1, 0, 1, 0, 1, 0, 0,
    1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0,
    1, 0, 1, 0, 1, 0, 0, 0, 0, 0,
};
static const uint32_t SOS_STEP_MS = 150;

}  // namespace board
