// Performance measurement: time per loop stage, loop jitter, overruns and CPU load.
// Stats are collected over a 1 s window, rollWindow() publishes them.
#pragma once

#include <stdint.h>

namespace perf {

enum class Stage : uint8_t {
    Rc,
    Imu,
    Attitude,
    Modes,
    Control,
    Mixer,
    Motors,
    Blackbox,
    SlowTasks,
    Count,
};

struct Stats {
    uint32_t minUs;
    uint32_t avgUs;
    uint32_t maxUs;
};

void init(uint32_t loopPeriodUs);

// Around the whole loop body. latenessUs is how late this tick started.
void loopStart(uint32_t latenessUs, uint32_t totalOverruns);
void loopEnd();

void begin(Stage stage);
void end(Stage stage);

void noteSdWrite(uint32_t durationUs);

// Publishes the current window and starts a new one. Call once per second.
void rollWindow();

// Published values from the last complete window.
const Stats& stage(Stage stage);
const Stats& loop();
uint32_t cpuLoadPermille();
uint32_t maxLatenessUs();
uint32_t overrunsInWindow();
uint32_t overrunsTotal();
uint32_t sdWriteMaxUs();
uint32_t lastLoopUs();
const char* stageName(Stage stage);

}  // namespace perf
