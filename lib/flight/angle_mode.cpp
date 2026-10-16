#include "angle_mode.h"

#include "fc_math.h"

RateSetpoint angleModeSetpoint(const RcInput& rc, const Angles& attitude,
                               const AngleModeConfig& config, const RatesConfig& rates)
{
    RateSetpoint target;

    float rollTarget = fcClamp(rc.stick[AXIS_ROLL], -1.0f, 1.0f) * config.maxAngleDeg;
    float pitchTarget = fcClamp(rc.stick[AXIS_PITCH], -1.0f, 1.0f) * config.maxAngleDeg;

    target.dps[AXIS_ROLL] = fcClamp((rollTarget - attitude.rollDeg) * config.angleP,
                                    -config.maxRateDps, config.maxRateDps);
    target.dps[AXIS_PITCH] = fcClamp((pitchTarget - attitude.pitchDeg) * config.angleP,
                                     -config.maxRateDps, config.maxRateDps);
    target.dps[AXIS_YAW] = stickToRate(rc.stick[AXIS_YAW], rates.maxDps[AXIS_YAW], rates.expo);
    return target;
}
