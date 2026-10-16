#include "pt1_filter.h"

#include "fc_math.h"

Pt1Filter::Pt1Filter()
    : gain_(1.0f), state_(0.0f)
{
}

void Pt1Filter::configure(float cutoffHz, float sampleHz)
{
    if (cutoffHz <= 0.0f || sampleHz <= 0.0f) {
        gain_ = 1.0f;
        return;
    }
    float rc = 1.0f / (2.0f * FC_PI * cutoffHz);
    float dt = 1.0f / sampleHz;
    gain_ = dt / (rc + dt);
}

float Pt1Filter::apply(float input)
{
    state_ += gain_ * (input - state_);
    return state_;
}

void Pt1Filter::reset(float value)
{
    state_ = value;
}
