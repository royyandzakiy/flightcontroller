#include "runaway_detector.h"

RunawayDetector::RunawayDetector()
    : errorThresholdDps_(200.0f), tripMs_(1000), active_(false), activeSinceMs_(0)
{
}

void RunawayDetector::configure(float errorThresholdDps, uint32_t tripMs)
{
    errorThresholdDps_ = errorThresholdDps;
    tripMs_ = tripMs;
}

bool RunawayDetector::update(bool armed, float maxAbsErrorDps, bool saturated, uint32_t nowMs)
{
    bool suspicious = armed && saturated && maxAbsErrorDps > errorThresholdDps_;
    if (!suspicious) {
        active_ = false;
        return false;
    }
    if (!active_) {
        active_ = true;
        activeSinceMs_ = nowMs;
    }
    return (nowMs - activeSinceMs_) >= tripMs_;
}

void RunawayDetector::reset()
{
    active_ = false;
}
