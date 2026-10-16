// Detects a quad that fights itself: control output saturated while the rate error
// stays large, e.g. wrong motor order, wrong props or a bad IMU orientation.
#pragma once

#include <stdint.h>

class RunawayDetector {
public:
    RunawayDetector();

    void configure(float errorThresholdDps, uint32_t tripMs);

    // Returns true once the condition has held for tripMs while armed.
    bool update(bool armed, float maxAbsErrorDps, bool saturated, uint32_t nowMs);
    void reset();

private:
    float errorThresholdDps_;
    uint32_t tripMs_;
    bool active_;
    uint32_t activeSinceMs_;
};
