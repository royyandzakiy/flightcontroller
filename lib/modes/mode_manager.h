// Decides what the quad is allowed to do: arming, flight mode and failsafe.
// Pure logic with time passed in, so every rule can be unit-tested on the PC.
#pragma once

#include <stdint.h>
#include "flight_types.h"

static const int AUX_ARM = 0;          // aux channel 1: arm switch
static const int AUX_FLIGHT_MODE = 1;  // aux channel 2: low = acro, mid/high = angle

enum class ArmState : uint8_t { Disarmed, Armed, Failsafe };
enum class FlightMode : uint8_t { Acro, Angle };
enum class DisarmReason : uint8_t { None, Switch, Failsafe, ImuFault, Runaway };

enum class ModeEvent : uint8_t {
    None,
    Armed,
    ArmBlocked,
    Disarmed,
    FailsafeStarted,
    FailsafeRecovered,
    FlightModeChanged,
};

// Every reason arming is refused. Several can be set at once.
enum ArmBlockFlag {
    ARM_BLOCK_NO_LINK = 1 << 0,
    ARM_BLOCK_THROTTLE_HIGH = 1 << 1,
    ARM_BLOCK_SWITCH_NOT_CYCLED = 1 << 2,
    ARM_BLOCK_NOT_CALIBRATED = 1 << 3,
    ARM_BLOCK_IMU_FAULT = 1 << 4,
    ARM_BLOCK_TILTED = 1 << 5,
    ARM_BLOCK_CLI_ACTIVE = 1 << 6,
    ARM_BLOCK_BATTERY_CRITICAL = 1 << 7,
    ARM_BLOCK_LOOP_OVERLOAD = 1 << 8,
    ARM_BLOCK_BOOT_GRACE = 1 << 9,
    ARM_BLOCK_FLAG_COUNT = 10,
};

struct ModeConfig {
    float armThrottleMax;        // 0..1
    float armMaxTiltDeg;
    uint32_t bootGraceMs;
    uint32_t failsafeStage1Ms;   // how long to hold level before cutting the motors
    float failsafeThrottle;      // 0..1 during stage 1
};

struct ModeInputs {
    uint32_t nowMs;
    RcInput rc;
    bool linkOk;
    bool gyroCalibrated;
    bool imuHealthy;
    float tiltDeg;
    bool batteryCritical;
    bool cliActive;
    bool loopOverloaded;
    bool runawayDetected;
};

class ModeManager {
public:
    ModeManager();

    void configure(const ModeConfig& config);

    // Call once per loop. Returns the most important thing that happened this call.
    ModeEvent update(const ModeInputs& in);

    ArmState armState() const { return state_; }
    // Motors may spin: armed or in failsafe stage 1.
    bool isArmed() const { return state_ != ArmState::Disarmed; }
    bool inFailsafe() const { return state_ == ArmState::Failsafe; }
    // Failsafe always flies in angle mode, to level the quad.
    FlightMode flightMode() const;
    float throttle() const { return throttle_; }
    uint16_t armBlockFlags() const { return armBlockFlags_; }
    DisarmReason lastDisarmReason() const { return lastDisarmReason_; }
    const char* flightModeName() const;

    static const char* armBlockFlagName(int bit);
    static const char* disarmReasonName(DisarmReason reason);

private:
    uint16_t computeArmBlockFlags(const ModeInputs& in) const;
    ModeEvent disarm(DisarmReason reason);

    ModeConfig config_;
    ArmState state_;
    FlightMode selectedMode_;
    DisarmReason lastDisarmReason_;
    uint16_t armBlockFlags_;
    bool armSwitchReady_;      // switch seen off with a working link since the last disarm
    bool lastArmSwitchOn_;
    uint32_t failsafeStartMs_;
    float throttle_;
};
