#include "rate_controller.h"

#include <math.h>

void RateController::configure(const RateControllerConfig& config)
{
    for (int axis = 0; axis < AXIS_COUNT; axis++) {
        pids_[axis].configure(config.gains[axis], config.dtermCutoffHz, config.loopHz);
    }
    maxAbsErrorDps_ = 0.0f;
    saturated_ = false;
}

AxisTorque RateController::update(const RateSetpoint& target, const float gyroDps[AXIS_COUNT],
                                  float dt, bool integrate)
{
    AxisTorque torque;
    maxAbsErrorDps_ = 0.0f;
    saturated_ = false;

    for (int axis = 0; axis < AXIS_COUNT; axis++) {
        torque.axis[axis] = pids_[axis].update(target.dps[axis], gyroDps[axis], dt, integrate);

        float absError = fabsf(target.dps[axis] - gyroDps[axis]);
        if (absError > maxAbsErrorDps_) maxAbsErrorDps_ = absError;
        if (fabsf(torque.axis[axis]) >= 0.999f) saturated_ = true;
    }
    return torque;
}

void RateController::reset()
{
    for (int axis = 0; axis < AXIS_COUNT; axis++) pids_[axis].reset();
}

void RateController::resetIntegrals()
{
    for (int axis = 0; axis < AXIS_COUNT; axis++) pids_[axis].resetIntegral();
}

uint32_t RateController::faults() const
{
    uint32_t total = 0;
    for (int axis = 0; axis < AXIS_COUNT; axis++) total += pids_[axis].faults();
    return total;
}
