// Angle (self-level) mode: stick sets a target angle, an outer P loop turns the
// angle error into a rate setpoint for the rate controller. Yaw stays rate-controlled.
#pragma once

#include "flight_types.h"
#include "rates.h"

struct AngleModeConfig {
    float maxAngleDeg;  // tilt at full stick
    float angleP;       // deg/s of rate per degree of angle error
    float maxRateDps;   // limit on the roll/pitch rate this loop may ask for
};

RateSetpoint angleModeSetpoint(const RcInput& rc, const Angles& attitude,
                               const AngleModeConfig& config, const RatesConfig& rates);
