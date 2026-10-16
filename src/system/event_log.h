// Event log: things that happened (arm, failsafe, faults...) with a timestamp.
// post() is cheap enough for the control loop, formatting happens later in flush().
// Every event goes to the blackbox, the console only shows levels up to the set level.
#pragma once

#include <stdint.h>

enum class LogLevel : uint8_t { Error = 0, Warn = 1, Info = 2, Debug = 3 };

enum class LogCode : uint8_t {
    Boot,
    WatchdogReset,
    ImuFound,
    ImuFault,
    GyroCalibrated,
    GyroCalibrationRetry,
    AccelCalibrated,
    Armed,
    ArmBlocked,
    Disarmed,
    FlightMode,
    FailsafeStarted,
    FailsafeRecovered,
    BatteryState,
    RcLinkLost,
    RcLinkRestored,
    SdReady,
    SdError,
    BlackboxStarted,
    BlackboxStopped,
    BlackboxFull,
    LoopOverrun,
    PidFault,
    ParamsLoaded,
    ParamsDefaults,
    ParamsSaved,
    CliOpened,
    CliClosed,
    MotorTest,
    Count,
};

namespace eventLog {

void init();
void post(LogLevel level, LogCode code, int32_t a = 0, int32_t b = 0);

// Formats a few queued events to the console and the blackbox. Call from a slow task.
void flush();
bool pending();

void setConsoleLevel(LogLevel level);
uint32_t droppedEvents();

}  // namespace eventLog
