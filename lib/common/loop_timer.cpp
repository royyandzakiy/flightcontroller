#include "loop_timer.h"

LoopTimer::LoopTimer()
    : periodUs_(1000), nextTickUs_(0), lastLatenessUs_(0), overruns_(0)
{
}

void LoopTimer::init(uint32_t periodUs, uint32_t nowUs)
{
    periodUs_ = periodUs;
    nextTickUs_ = nowUs + periodUs;
    lastLatenessUs_ = 0;
    overruns_ = 0;
}

bool LoopTimer::isTickDue(uint32_t nowUs)
{
    // Signed difference keeps this correct when micros() wraps after ~71 minutes.
    if ((int32_t)(nowUs - nextTickUs_) < 0) {
        return false;
    }

    lastLatenessUs_ = nowUs - nextTickUs_;
    if (lastLatenessUs_ >= periodUs_) {
        overruns_++;
        nextTickUs_ = nowUs + periodUs_;
    } else {
        nextTickUs_ += periodUs_;
    }
    return true;
}
