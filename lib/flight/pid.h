// PID controller for one axis of rotation rate.
// Inputs in deg/s, output is a normalized torque command (-1..1).
#pragma once

#include <stdint.h>
#include "pt1_filter.h"

struct PidGains {
    float p;       // per deg/s of error
    float i;       // per deg/s of error per second
    float d;       // per deg/s^2 of measured rate change
    float iLimit;  // I-term clamp, anti-windup
};

struct PidTerms {
    float p;
    float i;
    float d;
    float sum;  // clamped output
};

class Pid {
public:
    Pid();

    void configure(const PidGains& gains, float dtermCutoffHz, float loopHz);

    // integrate=false freezes the I-term (on the ground, at low throttle).
    float update(float setpointDps, float measuredDps, float dt, bool integrate);

    // Clears the I-term and the D-term history.
    void reset();
    void resetIntegral() { integral_ = 0.0f; }

    const PidTerms& terms() const { return terms_; }
    // Number of times the output was NaN/inf and got discarded.
    uint32_t faults() const { return faults_; }

private:
    PidGains gains_;
    Pt1Filter dtermFilter_;
    float integral_;
    float lastMeasurement_;
    bool hasLastMeasurement_;
    PidTerms terms_;
    uint32_t faults_;
};
