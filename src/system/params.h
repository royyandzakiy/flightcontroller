// Tunable parameters: one table of named floats with limits and defaults,
// saved to flash with a version and CRC. Change them with the CLI (get/set/save).
#pragma once

#include <stdint.h>

struct ParamValues {
    float rollP, rollI, rollD;
    float pitchP, pitchI, pitchD;
    float yawP, yawI, yawD;
    float iLimit;
    float gyroLpfHz;
    float dtermLpfHz;
    float rateMaxRollPitch;
    float rateMaxYaw;
    float rateExpo;
    float angleMax;
    float angleP;
    float angleMaxRate;
    float motorIdle;
    float mixYawReversed;     // 0 or 1
    float armThrottleMax;
    float armMaxAngle;
    float failsafeStage1Ms;
    float failsafeThrottle;
    float batteryDivider;     // battery volts per ADC pin volt
    float batteryCellLow;
    float batteryCellCritical;
    float accelOffsetX;       // g, from "calib accel"
    float accelOffsetY;
    float accelOffsetZ;
    float runawayErrorDps;
    float runawayMs;
    float logLevel;           // 0 error, 1 warn, 2 info, 3 debug
};

struct ParamDef {
    const char* name;
    float* value;
    float minValue;
    float maxValue;
    float defaultValue;
};

namespace params {

// Loads saved values, falls back to defaults if missing or corrupt.
void init();

const ParamValues& get();
ParamValues& edit();  // for calibration results; call markChanged() afterwards

int count();
const ParamDef& def(int index);
const ParamDef* find(const char* name);

// Returns false for an unknown name or a value outside the limits.
bool set(const char* name, float value);
void markChanged();
void resetToDefaults();
bool save();

// Increments on every change, so modules know when to re-read their settings.
uint32_t revision();

}  // namespace params
