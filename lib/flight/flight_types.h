// Data passed between the flight modules, in physical units.
//
// Body frame: x forward, y right, z down.
// Positive roll = right side down, positive pitch = nose up, positive yaw = nose right.
#pragma once

#include <stdint.h>

enum Axis { AXIS_ROLL = 0, AXIS_PITCH = 1, AXIS_YAW = 2, AXIS_COUNT = 3 };

static const int MOTOR_COUNT = 4;
static const int RC_AUX_COUNT = 4;

enum class SwitchPos : uint8_t { Low, Mid, High };

struct ImuSample {
    float gyroDps[AXIS_COUNT];     // filtered, used by the controller
    float gyroRawDps[AXIS_COUNT];  // unfiltered, for the blackbox
    float accelG[AXIS_COUNT];      // specific force in g, z reads -1 when level
};

struct RcInput {
    // -1..1 in body-axis sign: roll right, pitch nose up (stick pulled back), yaw nose right.
    float stick[AXIS_COUNT];
    float throttle;  // 0..1
    SwitchPos aux[RC_AUX_COUNT];
};

struct RateSetpoint {
    float dps[AXIS_COUNT];
};

// Controller output per axis, -1..1, before mixing.
struct AxisTorque {
    float axis[AXIS_COUNT];
};

struct MotorOutputs {
    float motor[MOTOR_COUNT];  // 0..1

    static MotorOutputs stopped()
    {
        MotorOutputs out;
        for (int i = 0; i < MOTOR_COUNT; i++) out.motor[i] = 0.0f;
        return out;
    }
};

struct Angles {
    float rollDeg;
    float pitchDeg;
};
