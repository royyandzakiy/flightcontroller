#include "system/params.h"

#include <string.h>

#include "board/board.h"
#include "byte_io.h"
#include "crc.h"
#include "system/event_log.h"

namespace {

ParamValues values;
uint32_t changeCounter = 1;

// PID gains are in normalized torque per deg/s: P 0.002 means 100 deg/s of error asks
// for 20% of the available torque. Starting points only, tune them from the blackbox.
ParamDef table[] = {
    { "roll_p", &values.rollP, 0.0f, 0.05f, 0.0020f },
    { "roll_i", &values.rollI, 0.0f, 0.1f, 0.0030f },
    { "roll_d", &values.rollD, 0.0f, 0.001f, 0.00002f },
    { "pitch_p", &values.pitchP, 0.0f, 0.05f, 0.0022f },
    { "pitch_i", &values.pitchI, 0.0f, 0.1f, 0.0030f },
    { "pitch_d", &values.pitchD, 0.0f, 0.001f, 0.000022f },
    { "yaw_p", &values.yawP, 0.0f, 0.05f, 0.0030f },
    { "yaw_i", &values.yawI, 0.0f, 0.1f, 0.0030f },
    { "yaw_d", &values.yawD, 0.0f, 0.001f, 0.0f },
    { "i_limit", &values.iLimit, 0.0f, 1.0f, 0.3f },
    { "gyro_lpf_hz", &values.gyroLpfHz, 0.0f, 500.0f, 90.0f },
    { "dterm_lpf_hz", &values.dtermLpfHz, 0.0f, 500.0f, 80.0f },
    { "rate_max_rp", &values.rateMaxRollPitch, 50.0f, 1500.0f, 500.0f },
    { "rate_max_y", &values.rateMaxYaw, 50.0f, 1500.0f, 400.0f },
    { "rate_expo", &values.rateExpo, 0.0f, 1.0f, 0.3f },
    { "angle_max", &values.angleMax, 10.0f, 80.0f, 45.0f },
    { "angle_p", &values.angleP, 0.5f, 20.0f, 5.0f },
    { "angle_max_rate", &values.angleMaxRate, 50.0f, 1000.0f, 300.0f },
    { "motor_idle", &values.motorIdle, 0.0f, 0.2f, 0.05f },
    { "mix_yaw_rev", &values.mixYawReversed, 0.0f, 1.0f, 0.0f },
    { "arm_thr_max", &values.armThrottleMax, 0.0f, 0.2f, 0.05f },
    { "arm_max_angle", &values.armMaxAngle, 5.0f, 180.0f, 25.0f },
    { "fs_stage1_ms", &values.failsafeStage1Ms, 0.0f, 10000.0f, 1000.0f },
    { "fs_throttle", &values.failsafeThrottle, 0.0f, 0.8f, 0.0f },
    { "bat_divider", &values.batteryDivider, 1.0f, 30.0f, 11.0f },
    { "bat_cell_low", &values.batteryCellLow, 3.0f, 4.2f, 3.5f },
    { "bat_cell_crit", &values.batteryCellCritical, 2.8f, 4.0f, 3.3f },
    { "acc_off_x", &values.accelOffsetX, -0.5f, 0.5f, 0.0f },
    { "acc_off_y", &values.accelOffsetY, -0.5f, 0.5f, 0.0f },
    { "acc_off_z", &values.accelOffsetZ, -0.5f, 0.5f, 0.0f },
    { "runaway_dps", &values.runawayErrorDps, 50.0f, 2000.0f, 300.0f },
    { "runaway_ms", &values.runawayMs, 100.0f, 10000.0f, 1000.0f },
    { "log_level", &values.logLevel, 0.0f, 3.0f, 2.0f },
};

const int PARAM_COUNT = sizeof(table) / sizeof(table[0]);

// Bump when the table changes order or meaning, old saves are then ignored.
const uint16_t STORAGE_VERSION = 1;
const uint32_t STORAGE_MAGIC = 0x46435031;  // "FCP1"
// magic + version + count + floats + crc
const uint32_t BLOB_SIZE = 4 + 2 + 2 + PARAM_COUNT * 4 + 2;

uint32_t floatBits(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

float bitsToFloat(uint32_t bits)
{
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

bool load()
{
    uint8_t blob[BLOB_SIZE];
    if (!board::storageLoad(blob, BLOB_SIZE)) return false;

    uint16_t storedCrc = (uint16_t)(blob[BLOB_SIZE - 2] | (blob[BLOB_SIZE - 1] << 8));
    if (crc16Ccitt(blob, BLOB_SIZE - 2) != storedCrc) return false;

    ByteReader reader(blob, BLOB_SIZE - 2);
    if (reader.u32le() != STORAGE_MAGIC) return false;
    if (reader.u16le() != STORAGE_VERSION) return false;
    if (reader.u16le() != PARAM_COUNT) return false;

    float loaded[PARAM_COUNT];
    for (int i = 0; i < PARAM_COUNT; i++) {
        loaded[i] = bitsToFloat(reader.u32le());
        // Out-of-range values mean a corrupt or incompatible save, take nothing from it.
        if (!(loaded[i] >= table[i].minValue && loaded[i] <= table[i].maxValue)) return false;
    }
    for (int i = 0; i < PARAM_COUNT; i++) *table[i].value = loaded[i];
    return true;
}

}  // namespace

namespace params {

void init()
{
    resetToDefaults();
    if (load()) {
        eventLog::post(LogLevel::Info, LogCode::ParamsLoaded);
    } else {
        resetToDefaults();
        eventLog::post(LogLevel::Warn, LogCode::ParamsDefaults);
    }
    markChanged();
}

const ParamValues& get()
{
    return values;
}

ParamValues& edit()
{
    return values;
}

int count()
{
    return PARAM_COUNT;
}

const ParamDef& def(int index)
{
    return table[index];
}

const ParamDef* find(const char* name)
{
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(table[i].name, name) == 0) return &table[i];
    }
    return NULL;
}

bool set(const char* name, float value)
{
    const ParamDef* param = find(name);
    if (param == NULL) return false;
    if (!(value >= param->minValue && value <= param->maxValue)) return false;
    *param->value = value;
    markChanged();
    return true;
}

void markChanged()
{
    changeCounter++;
}

void resetToDefaults()
{
    for (int i = 0; i < PARAM_COUNT; i++) *table[i].value = table[i].defaultValue;
    markChanged();
}

bool save()
{
    uint8_t blob[BLOB_SIZE];
    ByteWriter writer(blob, BLOB_SIZE);
    writer.u32le(STORAGE_MAGIC);
    writer.u16le(STORAGE_VERSION);
    writer.u16le(PARAM_COUNT);
    for (int i = 0; i < PARAM_COUNT; i++) writer.u32le(floatBits(*table[i].value));
    writer.u16le(crc16Ccitt(blob, BLOB_SIZE - 2));
    if (writer.overflowed()) return false;

    bool ok = board::storageSave(blob, BLOB_SIZE);
    if (ok) eventLog::post(LogLevel::Info, LogCode::ParamsSaved);
    return ok;
}

uint32_t revision()
{
    return changeCounter;
}

}  // namespace params
