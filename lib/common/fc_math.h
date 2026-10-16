// Small math helpers shared by the flight code.
// Names carry an fc prefix so they can't clash with min/max/constrain macros from Arduino.h.
#pragma once

#include <stdint.h>
#include <math.h>

static const float FC_PI = 3.14159265f;
static const float FC_DEG_TO_RAD = 0.0174532925f;
static const float FC_RAD_TO_DEG = 57.2957795f;

inline float fcClamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

inline int32_t fcClampInt(int32_t value, int32_t low, int32_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

// False for NaN and infinity. GCC builtin, so it works the same on every core.
inline bool fcIsFinite(float value)
{
    return __builtin_isfinite(value);
}

// Rounds to the nearest integer, halves away from zero.
inline int32_t fcRound(float value)
{
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}
