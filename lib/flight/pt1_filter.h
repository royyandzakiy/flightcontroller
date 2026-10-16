// First-order low-pass filter, the RC filter in software.
#pragma once

class Pt1Filter {
public:
    Pt1Filter();

    // A cutoff of 0 or less turns the filter into a pass-through.
    void configure(float cutoffHz, float sampleHz);
    float apply(float input);
    void reset(float value);
    float value() const { return state_; }

private:
    float gain_;
    float state_;
};
