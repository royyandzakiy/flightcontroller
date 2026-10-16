// Quad-X mixer: turns throttle + roll/pitch/yaw torque into four motor commands.
//
// Betaflight motor order, seen from above, props in:
//   4 front-left (CW)    2 front-right (CCW)
//   3 rear-left  (CCW)   1 rear-right  (CW)
#pragma once

#include "flight_types.h"

struct MixerConfig {
    bool yawReversed;  // true for props-out
    float idle;        // 0..1, armed minimum so props keep spinning
};

// Per motor: how much roll, pitch and yaw torque raise its command.
struct MotorMix {
    float roll;
    float pitch;
    float yaw;
};

const MotorMix* mixerQuadX();

// throttle 0..1, torque -1..1 per axis. Output 0..1 per motor, idle included.
MotorOutputs mixerMix(const AxisTorque& torque, float throttle, const MixerConfig& config);
