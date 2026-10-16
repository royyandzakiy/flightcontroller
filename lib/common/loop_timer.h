// Fixed-rate tick source for the main loop. Ticks are scheduled on a fixed grid,
// so a slightly late tick doesn't shift all later ones (no drift).
#pragma once

#include <stdint.h>

class LoopTimer {
public:
    LoopTimer();

    void init(uint32_t periodUs, uint32_t nowUs);

    // True once per period. Call it as often as possible.
    // A tick that starts a whole period late counts as an overrun and the grid restarts from now.
    bool isTickDue(uint32_t nowUs);

    uint32_t periodUs() const { return periodUs_; }
    uint32_t lastLatenessUs() const { return lastLatenessUs_; }
    uint32_t overruns() const { return overruns_; }

private:
    uint32_t periodUs_;
    uint32_t nextTickUs_;
    uint32_t lastLatenessUs_;
    uint32_t overruns_;
};
