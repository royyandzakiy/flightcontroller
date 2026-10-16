// Acro mode: stick position -> rotation rate.
#pragma once

#include "flight_types.h"

struct RatesConfig {
    float maxDps[AXIS_COUNT];  // rate at full stick
    float expo;                // 0 = linear, 1 = fully cubic (soft center)
};

// rate = maxDps * (expo * stick^3 + (1 - expo) * stick)
float stickToRate(float stick, float maxDps, float expo);

RateSetpoint ratesFromSticks(const RcInput& rc, const RatesConfig& config);
