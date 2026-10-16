// Inner control loop: one rate PID per axis, turns a rate setpoint into torque commands.
#pragma once

#include "flight_types.h"
#include "pid.h"

struct RateControllerConfig {
    PidGains gains[AXIS_COUNT];
    float dtermCutoffHz;
    float loopHz;
};

class RateController {
public:
    void configure(const RateControllerConfig& config);

    AxisTorque update(const RateSetpoint& target, const float gyroDps[AXIS_COUNT],
                      float dt, bool integrate);

    void reset();
    void resetIntegrals();

    const PidTerms& terms(int axis) const { return pids_[axis].terms(); }
    uint32_t faults() const;

    // Largest |setpoint - gyro| of the last update, for runaway detection.
    float maxAbsErrorDps() const { return maxAbsErrorDps_; }
    // True if any axis output hit the -1/1 limit in the last update.
    bool saturated() const { return saturated_; }

private:
    Pid pids_[AXIS_COUNT];
    float maxAbsErrorDps_;
    bool saturated_;
};
