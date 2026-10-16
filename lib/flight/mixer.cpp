#include "mixer.h"

#include "fc_math.h"

// Roll right = left motors up. Pitch nose up = front motors up.
// Yaw nose right = CCW props up, their reaction torque turns the frame clockwise.
static const MotorMix QUAD_X[MOTOR_COUNT] = {
    { -1.0f, -1.0f, -1.0f },  // 1 rear-right, CW
    { -1.0f,  1.0f,  1.0f },  // 2 front-right, CCW
    {  1.0f, -1.0f,  1.0f },  // 3 rear-left, CCW
    {  1.0f,  1.0f, -1.0f },  // 4 front-left, CW
};

const MotorMix* mixerQuadX()
{
    return QUAD_X;
}

MotorOutputs mixerMix(const AxisTorque& torque, float throttle, const MixerConfig& config)
{
    float yawSign = config.yawReversed ? -1.0f : 1.0f;
    float rpy[MOTOR_COUNT];
    float rpyMin = 0.0f;
    float rpyMax = 0.0f;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        rpy[i] = QUAD_X[i].roll * torque.axis[AXIS_ROLL]
               + QUAD_X[i].pitch * torque.axis[AXIS_PITCH]
               + QUAD_X[i].yaw * yawSign * torque.axis[AXIS_YAW];
        if (i == 0 || rpy[i] < rpyMin) rpyMin = rpy[i];
        if (i == 0 || rpy[i] > rpyMax) rpyMax = rpy[i];
    }

    // More spread than the motors can deliver: scale the attitude part down evenly.
    float range = rpyMax - rpyMin;
    if (range > 1.0f) {
        for (int i = 0; i < MOTOR_COUNT; i++) rpy[i] /= range;
        rpyMin /= range;
        rpyMax /= range;
    }

    // Shift throttle so every motor stays in 0..1: attitude control beats exact throttle.
    float shiftedThrottle = fcClamp(throttle, -rpyMin, 1.0f - rpyMax);

    MotorOutputs out;
    float idle = fcClamp(config.idle, 0.0f, 0.5f);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        float command = fcClamp(shiftedThrottle + rpy[i], 0.0f, 1.0f);
        out.motor[i] = idle + command * (1.0f - idle);
    }
    return out;
}
