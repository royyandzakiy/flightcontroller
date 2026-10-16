#include "drivers/battery.h"

#include "board/board.h"
#include "pt1_filter.h"
#include "system/event_log.h"

namespace {

const float UPDATE_HZ = 50.0f;
const float FILTER_CUTOFF_HZ = 0.5f;   // smooths out motor-load sag spikes
const float ABSENT_BELOW_VOLTS = 2.0f; // USB power only, no flight battery
const float CELL_MAX_VOLTS = 4.35f;    // HV LiPo full charge
const uint32_t SETTLE_SAMPLES = 100;   // 2 s for the filter before trusting the cell count
// Recovering to a better state needs this much margin, so a voltage sitting on the
// threshold doesn't flip the state back and forth.
const float RECOVERY_MARGIN_VOLTS = 0.1f;

Pt1Filter filter;
float dividerScale = 11.0f;
float cellLow = 3.5f;
float cellCritical = 3.3f;
float filteredVolts = 0.0f;
uint8_t cellCount = 0;
uint32_t samplesWithBattery = 0;
BatteryState currentState = BatteryState::Absent;
bool started = false;

BatteryState evaluate(float perCell)
{
    switch (currentState) {
    case BatteryState::Critical:
        if (perCell > cellCritical + RECOVERY_MARGIN_VOLTS) return BatteryState::Low;
        return BatteryState::Critical;
    case BatteryState::Low:
        if (perCell < cellCritical) return BatteryState::Critical;
        if (perCell > cellLow + RECOVERY_MARGIN_VOLTS) return BatteryState::Ok;
        return BatteryState::Low;
    default:
        if (perCell < cellCritical) return BatteryState::Critical;
        if (perCell < cellLow) return BatteryState::Low;
        return BatteryState::Ok;
    }
}

}  // namespace

namespace battery {

void configure(float scale, float cellLowVolts, float cellCriticalVolts)
{
    dividerScale = scale;
    cellLow = cellLowVolts;
    cellCritical = cellCriticalVolts;
    filter.configure(FILTER_CUTOFF_HZ, UPDATE_HZ);
}

void update()
{
    float measured = board::batteryPinVolts() * dividerScale;
    if (!started) {
        filter.reset(measured);
        started = true;
    }
    filteredVolts = filter.apply(measured);

    BatteryState next;
    if (filteredVolts < ABSENT_BELOW_VOLTS) {
        cellCount = 0;
        samplesWithBattery = 0;
        next = BatteryState::Absent;
    } else {
        samplesWithBattery++;
        if (cellCount == 0) {
            if (samplesWithBattery < SETTLE_SAMPLES) return;
            cellCount = (uint8_t)(filteredVolts / CELL_MAX_VOLTS) + 1;
        }
        next = evaluate(filteredVolts / cellCount);
    }

    if (next != currentState) {
        currentState = next;
        LogLevel level = (next == BatteryState::Low || next == BatteryState::Critical) ? LogLevel::Warn
                                                                                       : LogLevel::Info;
        eventLog::post(level, LogCode::BatteryState, (int32_t)next, (int32_t)millivolts());
    }
}

float volts()
{
    return filteredVolts;
}

uint16_t millivolts()
{
    float mv = filteredVolts * 1000.0f;
    if (mv < 0.0f) return 0;
    if (mv > 65535.0f) return 65535;
    return (uint16_t)mv;
}

uint8_t cells()
{
    return cellCount;
}

BatteryState state()
{
    return currentState;
}

const char* stateName(BatteryState value)
{
    switch (value) {
    case BatteryState::Absent: return "absent";
    case BatteryState::Ok: return "ok";
    case BatteryState::Low: return "LOW";
    case BatteryState::Critical: return "CRITICAL";
    }
    return "?";
}

}  // namespace battery
