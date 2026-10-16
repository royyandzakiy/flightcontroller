#include "pid.h"

#include "fc_math.h"

Pid::Pid()
    : integral_(0.0f), lastMeasurement_(0.0f), hasLastMeasurement_(false), faults_(0)
{
    gains_.p = 0.0f;
    gains_.i = 0.0f;
    gains_.d = 0.0f;
    gains_.iLimit = 0.0f;
    terms_.p = 0.0f;
    terms_.i = 0.0f;
    terms_.d = 0.0f;
    terms_.sum = 0.0f;
}

void Pid::configure(const PidGains& gains, float dtermCutoffHz, float loopHz)
{
    gains_ = gains;
    dtermFilter_.configure(dtermCutoffHz, loopHz);
}

void Pid::reset()
{
    integral_ = 0.0f;
    hasLastMeasurement_ = false;
    dtermFilter_.reset(0.0f);
    terms_.p = 0.0f;
    terms_.i = 0.0f;
    terms_.d = 0.0f;
    terms_.sum = 0.0f;
}

float Pid::update(float setpointDps, float measuredDps, float dt, bool integrate)
{
    float error = setpointDps - measuredDps;

    float p = gains_.p * error;

    if (integrate) {
        integral_ = fcClamp(integral_ + gains_.i * error * dt, -gains_.iLimit, gains_.iLimit);
    }

    // D on the measurement, not the error, so a sudden stick move doesn't kick the output.
    float dRaw = 0.0f;
    if (hasLastMeasurement_ && dt > 0.0f) {
        dRaw = -gains_.d * (measuredDps - lastMeasurement_) / dt;
    }
    lastMeasurement_ = measuredDps;
    hasLastMeasurement_ = true;
    float d = dtermFilter_.apply(dRaw);

    float sum = p + integral_ + d;
    if (!fcIsFinite(sum)) {
        // A NaN would otherwise stick in the integral and the filter forever.
        faults_++;
        reset();
        return 0.0f;
    }

    terms_.p = p;
    terms_.i = integral_;
    terms_.d = d;
    terms_.sum = fcClamp(sum, -1.0f, 1.0f);
    return terms_.sum;
}
