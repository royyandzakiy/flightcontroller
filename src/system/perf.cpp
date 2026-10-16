#include "system/perf.h"

#include "board/board.h"

namespace {

const int STAGE_COUNT = (int)perf::Stage::Count;

struct Accumulator {
    uint32_t minUs;
    uint32_t maxUs;
    uint32_t totalUs;
    uint32_t count;
};

const char* const STAGE_NAMES[STAGE_COUNT] = {
    "rc", "imu", "attitude", "modes", "control", "mixer", "motors", "blackbox", "slow",
};

uint32_t periodUs = 1000;

Accumulator stageAcc[STAGE_COUNT];
Accumulator loopAcc;
uint32_t stageStartUs[STAGE_COUNT];
uint32_t loopStartUs = 0;
uint32_t windowStartUs = 0;
uint32_t windowMaxLatenessUs = 0;
uint32_t windowStartOverruns = 0;
uint32_t windowSdWriteMaxUs = 0;
uint32_t latestOverrunsTotal = 0;
uint32_t latestLoopUs = 0;

perf::Stats stagePublished[STAGE_COUNT];
perf::Stats loopPublished;
uint32_t cpuLoadPublished = 0;
uint32_t maxLatenessPublished = 0;
uint32_t overrunsPublished = 0;
uint32_t sdWriteMaxPublished = 0;

void resetAccumulator(Accumulator& acc)
{
    acc.minUs = 0xFFFFFFFFu;
    acc.maxUs = 0;
    acc.totalUs = 0;
    acc.count = 0;
}

void addSample(Accumulator& acc, uint32_t us)
{
    if (us < acc.minUs) acc.minUs = us;
    if (us > acc.maxUs) acc.maxUs = us;
    acc.totalUs += us;
    acc.count++;
}

perf::Stats publish(const Accumulator& acc)
{
    perf::Stats stats;
    if (acc.count == 0) {
        stats.minUs = 0;
        stats.avgUs = 0;
        stats.maxUs = 0;
    } else {
        stats.minUs = acc.minUs;
        stats.avgUs = acc.totalUs / acc.count;
        stats.maxUs = acc.maxUs;
    }
    return stats;
}

}  // namespace

namespace perf {

void init(uint32_t loopPeriodUs)
{
    periodUs = loopPeriodUs;
    for (int i = 0; i < STAGE_COUNT; i++) {
        resetAccumulator(stageAcc[i]);
        stagePublished[i] = publish(stageAcc[i]);
    }
    resetAccumulator(loopAcc);
    loopPublished = publish(loopAcc);
    windowStartUs = board::micros();
}

void loopStart(uint32_t latenessUs, uint32_t totalOverruns)
{
    loopStartUs = board::micros();
    if (latenessUs > windowMaxLatenessUs) windowMaxLatenessUs = latenessUs;
    latestOverrunsTotal = totalOverruns;
}

void loopEnd()
{
    latestLoopUs = board::micros() - loopStartUs;
    addSample(loopAcc, latestLoopUs);
}

void begin(Stage stage)
{
    stageStartUs[(int)stage] = board::micros();
}

void end(Stage stage)
{
    addSample(stageAcc[(int)stage], board::micros() - stageStartUs[(int)stage]);
}

void noteSdWrite(uint32_t durationUs)
{
    if (durationUs > windowSdWriteMaxUs) windowSdWriteMaxUs = durationUs;
}

void rollWindow()
{
    uint32_t now = board::micros();
    uint32_t windowUs = now - windowStartUs;

    for (int i = 0; i < STAGE_COUNT; i++) {
        stagePublished[i] = publish(stageAcc[i]);
        resetAccumulator(stageAcc[i]);
    }
    loopPublished = publish(loopAcc);
    // Busy time over wall time. 64-bit so a long window can't overflow the multiply.
    cpuLoadPublished = windowUs > 0 ? (uint32_t)((uint64_t)loopAcc.totalUs * 1000u / windowUs) : 0;
    resetAccumulator(loopAcc);

    maxLatenessPublished = windowMaxLatenessUs;
    overrunsPublished = latestOverrunsTotal - windowStartOverruns;
    sdWriteMaxPublished = windowSdWriteMaxUs;

    windowStartUs = now;
    windowMaxLatenessUs = 0;
    windowStartOverruns = latestOverrunsTotal;
    windowSdWriteMaxUs = 0;
}

const Stats& stage(Stage stage)
{
    return stagePublished[(int)stage];
}

const Stats& loop()
{
    return loopPublished;
}

uint32_t cpuLoadPermille()
{
    return cpuLoadPublished;
}

uint32_t maxLatenessUs()
{
    return maxLatenessPublished;
}

uint32_t overrunsInWindow()
{
    return overrunsPublished;
}

uint32_t overrunsTotal()
{
    return latestOverrunsTotal;
}

uint32_t sdWriteMaxUs()
{
    return sdWriteMaxPublished;
}

uint32_t lastLoopUs()
{
    return latestLoopUs;
}

const char* stageName(Stage stage)
{
    return STAGE_NAMES[(int)stage];
}

}  // namespace perf
