#include "rates.h"

#include "fc_math.h"

float stickToRate(float stick, float maxDps, float expo)
{
    stick = fcClamp(stick, -1.0f, 1.0f);
    expo = fcClamp(expo, 0.0f, 1.0f);
    float shaped = expo * stick * stick * stick + (1.0f - expo) * stick;
    return maxDps * shaped;
}

RateSetpoint ratesFromSticks(const RcInput& rc, const RatesConfig& config)
{
    RateSetpoint target;
    for (int axis = 0; axis < AXIS_COUNT; axis++) {
        target.dps[axis] = stickToRate(rc.stick[axis], config.maxDps[axis], config.expo);
    }
    return target;
}
