#include "mode_manager.h"

ModeManager::ModeManager()
    : state_(ArmState::Disarmed),
      selectedMode_(FlightMode::Angle),
      lastDisarmReason_(DisarmReason::None),
      armBlockFlags_(0),
      armSwitchReady_(false),
      lastArmSwitchOn_(false),
      failsafeStartMs_(0),
      throttle_(0.0f)
{
    config_.armThrottleMax = 0.05f;
    config_.armMaxTiltDeg = 25.0f;
    config_.bootGraceMs = 2000;
    config_.failsafeStage1Ms = 1000;
    config_.failsafeThrottle = 0.0f;
}

void ModeManager::configure(const ModeConfig& config)
{
    config_ = config;
}

uint16_t ModeManager::computeArmBlockFlags(const ModeInputs& in) const
{
    uint16_t flags = 0;
    if (!in.linkOk) flags |= ARM_BLOCK_NO_LINK;
    if (in.rc.throttle > config_.armThrottleMax) flags |= ARM_BLOCK_THROTTLE_HIGH;
    if (!armSwitchReady_) flags |= ARM_BLOCK_SWITCH_NOT_CYCLED;
    if (!in.gyroCalibrated) flags |= ARM_BLOCK_NOT_CALIBRATED;
    if (!in.imuHealthy) flags |= ARM_BLOCK_IMU_FAULT;
    if (in.tiltDeg > config_.armMaxTiltDeg) flags |= ARM_BLOCK_TILTED;
    if (in.cliActive) flags |= ARM_BLOCK_CLI_ACTIVE;
    if (in.batteryCritical) flags |= ARM_BLOCK_BATTERY_CRITICAL;
    if (in.loopOverloaded) flags |= ARM_BLOCK_LOOP_OVERLOAD;
    if (in.nowMs < config_.bootGraceMs) flags |= ARM_BLOCK_BOOT_GRACE;
    return flags;
}

ModeEvent ModeManager::disarm(DisarmReason reason)
{
    state_ = ArmState::Disarmed;
    lastDisarmReason_ = reason;
    // Whatever the reason, the pilot has to flip the switch off and on again to re-arm.
    armSwitchReady_ = false;
    return ModeEvent::Disarmed;
}

ModeEvent ModeManager::update(const ModeInputs& in)
{
    // Without a link the switch position is unknown, so it counts as off.
    bool armSwitchOn = in.linkOk && in.rc.aux[AUX_ARM] == SwitchPos::High;
    if (in.linkOk && !armSwitchOn) {
        armSwitchReady_ = true;
    }
    armBlockFlags_ = computeArmBlockFlags(in);

    ModeEvent event = ModeEvent::None;

    switch (state_) {
    case ArmState::Disarmed:
        // Arm only on the off -> on edge, a switch left on never arms by itself.
        if (armSwitchOn && !lastArmSwitchOn_) {
            if (armBlockFlags_ == 0) {
                state_ = ArmState::Armed;
                event = ModeEvent::Armed;
            } else {
                // A refused attempt needs a fresh switch cycle, so the quad can't arm
                // later on its own when the blocking condition clears.
                armSwitchReady_ = false;
                event = ModeEvent::ArmBlocked;
            }
        }
        break;

    case ArmState::Armed:
        if (!in.imuHealthy) {
            event = disarm(DisarmReason::ImuFault);
        } else if (in.runawayDetected) {
            event = disarm(DisarmReason::Runaway);
        } else if (!in.linkOk) {
            state_ = ArmState::Failsafe;
            failsafeStartMs_ = in.nowMs;
            event = ModeEvent::FailsafeStarted;
        } else if (!armSwitchOn) {
            event = disarm(DisarmReason::Switch);
        }
        break;

    case ArmState::Failsafe:
        if (!in.imuHealthy) {
            event = disarm(DisarmReason::ImuFault);
        } else if (in.linkOk) {
            if (armSwitchOn) {
                state_ = ArmState::Armed;
                event = ModeEvent::FailsafeRecovered;
            } else {
                event = disarm(DisarmReason::Switch);
            }
        } else if (in.nowMs - failsafeStartMs_ >= config_.failsafeStage1Ms) {
            event = disarm(DisarmReason::Failsafe);
        }
        break;
    }

    if (in.linkOk) {
        FlightMode wanted = (in.rc.aux[AUX_FLIGHT_MODE] == SwitchPos::Low) ? FlightMode::Acro
                                                                            : FlightMode::Angle;
        if (wanted != selectedMode_) {
            selectedMode_ = wanted;
            if (event == ModeEvent::None) event = ModeEvent::FlightModeChanged;
        }
    }

    switch (state_) {
    case ArmState::Armed:
        throttle_ = in.rc.throttle;
        break;
    case ArmState::Failsafe:
        throttle_ = config_.failsafeThrottle;
        break;
    case ArmState::Disarmed:
        throttle_ = 0.0f;
        break;
    }

    lastArmSwitchOn_ = armSwitchOn;
    return event;
}

FlightMode ModeManager::flightMode() const
{
    return (state_ == ArmState::Failsafe) ? FlightMode::Angle : selectedMode_;
}

const char* ModeManager::flightModeName() const
{
    if (state_ == ArmState::Failsafe) return "!FS!";
    return (selectedMode_ == FlightMode::Acro) ? "ACRO" : "ANGL";
}

const char* ModeManager::armBlockFlagName(int bit)
{
    static const char* const NAMES[ARM_BLOCK_FLAG_COUNT] = {
        "NO_LINK", "THROTTLE_HIGH", "SWITCH_NOT_CYCLED", "NOT_CALIBRATED", "IMU_FAULT",
        "TILTED", "CLI_ACTIVE", "BATTERY_CRITICAL", "LOOP_OVERLOAD", "BOOT_GRACE",
    };
    if (bit < 0 || bit >= ARM_BLOCK_FLAG_COUNT) return "?";
    return NAMES[bit];
}

const char* ModeManager::disarmReasonName(DisarmReason reason)
{
    switch (reason) {
    case DisarmReason::None: return "none";
    case DisarmReason::Switch: return "switch";
    case DisarmReason::Failsafe: return "failsafe";
    case DisarmReason::ImuFault: return "imu fault";
    case DisarmReason::Runaway: return "runaway";
    }
    return "?";
}
