// Quadcopter flight controller.
//
// One fixed-rate loop does everything that matters for flight, in this order:
//   1. read the radio and the IMU
//   2. decide what is allowed (arming, flight mode, failsafe)
//   3. compute the rotation rates we want and the torque to get there
//   4. mix torque into motor commands and send them
//   5. record what happened, then do one slow housekeeping job
// Each step lives in its own folder, this file shows the order and the reasons.

#include <Arduino.h>

#include "board/board.h"
#include "config.h"

#include "drivers/battery.h"
#include "drivers/imu_mpu6050.h"
#include "drivers/rc_crsf.h"
#include "drivers/status_led.h"

#include "system/blackbox.h"
#include "system/cli.h"
#include "system/console.h"
#include "system/event_log.h"
#include "system/params.h"
#include "system/perf.h"
#include "system/telemetry.h"

#include "angle_mode.h"
#include "attitude.h"
#include "loop_timer.h"
#include "mixer.h"
#include "mode_manager.h"
#include "rate_controller.h"
#include "rates.h"
#include "runaway_detector.h"

// All flight state is static: memory use is fixed at link time, nothing is allocated.
static LoopTimer loopTimer;
static AttitudeEstimator attitude;
static RateController rateController;
static RunawayDetector runaway;
static ModeManager modes;

// Settings copied from the parameters by applyParams().
static RatesConfig ratesConfig;
static AngleModeConfig angleConfig;
static MixerConfig mixerConfig;

static uint32_t appliedParamsRevision = 0;
static uint32_t lastLoopStartUs = 0;
static uint32_t lastPidFaults = 0;
static bool runawayDetected = false;

static void applyParams();
static float measureDt(uint32_t nowUs);
static ModeInputs collectModeInputs(uint32_t nowMs);
static void handleModeEvent(ModeEvent event, uint32_t nowMs);
static RateSetpoint computeSetpoint(const RcInput& rc);
static void writeMotors(const MotorOutputs& out, uint32_t nowMs);
static void recordLoop(const ImuSample& sample, const RateSetpoint& target, const MotorOutputs& out);
static void runSlowTasks(uint32_t nowMs);

void setup()
{
    board::init(DEBUG_BAUD);
    // Stop pulses first: ESCs must never see a floating or random signal.
    board::motorsInit();

    console::init();
    // Setup may wait on the UART, so no boot message is dropped. The loop never does.
    console::setBlocking(true);
    eventLog::init();
    eventLog::post(LogLevel::Info, LogCode::Boot, board::resetWasWatchdog() ? 1 : 0);
    if (board::resetWasWatchdog()) eventLog::post(LogLevel::Error, LogCode::WatchdogReset);
    console::printf("\r\nflight controller on %s, loop %u Hz, reset: %s\r\n",
                    board::info().name, (unsigned)board::info().loopHz, board::resetReason());

    params::init();

    if (!imu::init()) board::fatal("IMU not found, check wiring and I2C address");
    rc::init();
    blackbox::init();
    applyParams();

    // The gyro bias must be measured with the quad still. Retries until it is.
    console::print("calibrating gyro, keep still...\r\n");
    while (!imu::calibrateGyro()) {
        eventLog::post(LogLevel::Warn, LogCode::GyroCalibrationRetry);
        eventLog::flush();
        console::flushBlocking();
    }
    eventLog::post(LogLevel::Info, LogCode::GyroCalibrated);
    attitude.resetFromAccel(imu::read().accelG);

    cli::init(modes, attitude);
    while (eventLog::pending()) eventLog::flush();
    console::print("ready, type help for commands\r\n");
    console::flushBlocking();
    console::setBlocking(false);

    // From here on: no heap, and a hung loop resets the board.
    board::heapGuardArm();
    board::watchdogInit(WATCHDOG_TIMEOUT_MS);

    uint32_t periodUs = 1000000UL / board::info().loopHz;
    perf::init(periodUs);
    lastLoopStartUs = board::micros();
    loopTimer.init(periodUs, lastLoopStartUs);
}

void loop()
{
    uint32_t nowUs = board::micros();
    if (!loopTimer.isTickDue(nowUs)) return;
    uint32_t nowMs = board::millis();
    float dt = measureDt(nowUs);
    perf::loopStart(loopTimer.lastLatenessUs(), loopTimer.overruns());

    // 1. Inputs
    perf::begin(perf::Stage::Rc);
    rc::update(nowMs);
    perf::end(perf::Stage::Rc);

    perf::begin(perf::Stage::Imu);
    const ImuSample& sample = imu::read();
    perf::end(perf::Stage::Imu);

    perf::begin(perf::Stage::Attitude);
    attitude.update(sample, dt);
    perf::end(perf::Stage::Attitude);

    // 2. Decide what we are allowed to do
    perf::begin(perf::Stage::Modes);
    ModeEvent event = modes.update(collectModeInputs(nowMs));
    handleModeEvent(event, nowMs);
    perf::end(perf::Stage::Modes);

    // 3. Compute what we want: rates from the sticks (or from the angle error), then torque
    perf::begin(perf::Stage::Control);
    RateSetpoint target = computeSetpoint(rc::input());
    // Only integrate in flight: on the ground the error can't shrink and I would wind up.
    bool integrate = modes.isArmed() && modes.throttle() > PID_I_THROTTLE_MIN;
    AxisTorque torque = rateController.update(target, sample.gyroDps, dt, integrate);
    // Used by the mode manager on the next tick, one loop of delay doesn't matter here.
    runawayDetected = runaway.update(modes.isArmed(), rateController.maxAbsErrorDps(),
                                     rateController.saturated(), nowMs);
    perf::end(perf::Stage::Control);

    // 4. Act
    perf::begin(perf::Stage::Mixer);
    MotorOutputs out = modes.isArmed() ? mixerMix(torque, modes.throttle(), mixerConfig)
                                       : MotorOutputs::stopped();
    perf::end(perf::Stage::Mixer);

    perf::begin(perf::Stage::Motors);
    writeMotors(out, nowMs);
    perf::end(perf::Stage::Motors);

    // 5. Record and housekeeping
    perf::begin(perf::Stage::Blackbox);
    recordLoop(sample, target, out);
    perf::end(perf::Stage::Blackbox);

    perf::begin(perf::Stage::SlowTasks);
    runSlowTasks(nowMs);
    perf::end(perf::Stage::SlowTasks);

    board::watchdogKick();
    perf::loopEnd();
}

// Copies parameter values into the module configs. Runs at boot and after any change.
static void applyParams()
{
    const ParamValues& p = params::get();
    float loopHz = (float)board::info().loopHz;

    float accelOffset[AXIS_COUNT] = { p.accelOffsetX, p.accelOffsetY, p.accelOffsetZ };
    imu::configure(p.gyroLpfHz, loopHz, accelOffset);
    attitude.configure(ATTITUDE_ACCEL_TIME_CONSTANT_S);

    RateControllerConfig rate;
    PidGains roll = { p.rollP, p.rollI, p.rollD, p.iLimit };
    PidGains pitch = { p.pitchP, p.pitchI, p.pitchD, p.iLimit };
    PidGains yaw = { p.yawP, p.yawI, p.yawD, p.iLimit };
    rate.gains[AXIS_ROLL] = roll;
    rate.gains[AXIS_PITCH] = pitch;
    rate.gains[AXIS_YAW] = yaw;
    rate.dtermCutoffHz = p.dtermLpfHz;
    rate.loopHz = loopHz;
    rateController.configure(rate);

    ratesConfig.maxDps[AXIS_ROLL] = p.rateMaxRollPitch;
    ratesConfig.maxDps[AXIS_PITCH] = p.rateMaxRollPitch;
    ratesConfig.maxDps[AXIS_YAW] = p.rateMaxYaw;
    ratesConfig.expo = p.rateExpo;

    angleConfig.maxAngleDeg = p.angleMax;
    angleConfig.angleP = p.angleP;
    angleConfig.maxRateDps = p.angleMaxRate;

    mixerConfig.yawReversed = p.mixYawReversed > 0.5f;
    mixerConfig.idle = p.motorIdle;

    ModeConfig modeConfig;
    modeConfig.armThrottleMax = p.armThrottleMax;
    modeConfig.armMaxTiltDeg = p.armMaxAngle;
    modeConfig.bootGraceMs = ARM_BOOT_GRACE_MS;
    modeConfig.failsafeStage1Ms = (uint32_t)p.failsafeStage1Ms;
    modeConfig.failsafeThrottle = p.failsafeThrottle;
    modes.configure(modeConfig);

    runaway.configure(p.runawayErrorDps, (uint32_t)p.runawayMs);
    battery::configure(p.batteryDivider, p.batteryCellLow, p.batteryCellCritical);
    blackbox::setRateDivider(board::info().blackboxRateDiv);
    eventLog::setConsoleLevel((LogLevel)(int)p.logLevel);

    appliedParamsRevision = params::revision();
}

// Real time since the last tick, so a late tick doesn't make the PID think less time passed.
static float measureDt(uint32_t nowUs)
{
    float nominal = 1.0f / board::info().loopHz;
    float dt = (nowUs - lastLoopStartUs) * 1e-6f;
    lastLoopStartUs = nowUs;
    if (dt < nominal * 0.5f || dt > nominal * 4.0f) return nominal;
    return dt;
}

static ModeInputs collectModeInputs(uint32_t nowMs)
{
    ModeInputs in;
    in.nowMs = nowMs;
    in.rc = rc::input();
    in.linkOk = rc::linkOk();
    in.gyroCalibrated = imu::gyroCalibrated();
    in.imuHealthy = imu::healthy();
    in.tiltDeg = attitude.tiltDeg();
    in.batteryCritical = battery::state() == BatteryState::Critical;
    in.cliActive = cli::active();
    in.loopOverloaded = perf::overrunsInWindow() > LOOP_OVERLOAD_OVERRUNS_PER_S;
    in.runawayDetected = runawayDetected;
    return in;
}

static void handleModeEvent(ModeEvent event, uint32_t nowMs)
{
    switch (event) {
    case ModeEvent::Armed:
        // Start from a clean controller, nothing left over from the last flight.
        rateController.reset();
        runaway.reset();
        cli::stopMotorTest();
        blackbox::start(nowMs);
        eventLog::post(LogLevel::Info, LogCode::Armed);
        break;
    case ModeEvent::ArmBlocked:
        eventLog::post(LogLevel::Warn, LogCode::ArmBlocked, modes.armBlockFlags());
        break;
    case ModeEvent::Disarmed:
        rateController.reset();
        runawayDetected = false;
        blackbox::requestStop();
        eventLog::post(modes.lastDisarmReason() == DisarmReason::Switch ? LogLevel::Info : LogLevel::Error,
                       LogCode::Disarmed, (int32_t)modes.lastDisarmReason());
        break;
    case ModeEvent::FailsafeStarted:
        eventLog::post(LogLevel::Error, LogCode::FailsafeStarted);
        break;
    case ModeEvent::FailsafeRecovered:
        eventLog::post(LogLevel::Warn, LogCode::FailsafeRecovered);
        break;
    case ModeEvent::FlightModeChanged:
        // The I-term built up in one mode doesn't fit the other.
        rateController.resetIntegrals();
        eventLog::post(LogLevel::Info, LogCode::FlightMode, (int32_t)modes.flightMode());
        break;
    case ModeEvent::None:
        break;
    }
}

static RateSetpoint computeSetpoint(const RcInput& rc)
{
    if (modes.inFailsafe()) {
        // Sticks are unknown: hold level, the mode manager sets the throttle.
        RcInput level = rc;
        for (int axis = 0; axis < AXIS_COUNT; axis++) level.stick[axis] = 0.0f;
        return angleModeSetpoint(level, attitude.angles(), angleConfig, ratesConfig);
    }
    if (modes.flightMode() == FlightMode::Angle) {
        return angleModeSetpoint(rc, attitude.angles(), angleConfig, ratesConfig);
    }
    return ratesFromSticks(rc, ratesConfig);
}

static void writeMotors(const MotorOutputs& out, uint32_t nowMs)
{
    if (modes.isArmed()) {
        board::motorsWrite(out.motor);
        return;
    }
    // Disarmed: only a CLI motor test may spin a motor, one at a time.
    if (cli::motorTestActive(nowMs)) {
        MotorOutputs test = MotorOutputs::stopped();
        test.motor[cli::motorTestIndex()] = cli::motorTestValue();
        board::motorsWrite(test.motor);
        return;
    }
    board::motorsStop();
}

static void recordLoop(const ImuSample& sample, const RateSetpoint& target, const MotorOutputs& out)
{
    if (!blackbox::logging()) return;

    BbLoopRecord r;
    r.timeUs = board::micros();
    r.loopUs = (uint16_t)perf::lastLoopUs();
    for (int axis = 0; axis < AXIS_COUNT; axis++) {
        r.gyroRawDps[axis] = sample.gyroRawDps[axis];
        r.gyroDps[axis] = sample.gyroDps[axis];
        r.setpointDps[axis] = target.dps[axis];
        r.pidP[axis] = rateController.terms(axis).p;
        r.pidI[axis] = rateController.terms(axis).i;
        r.pidD[axis] = rateController.terms(axis).d;
        r.rcStick[axis] = rc::input().stick[axis];
    }
    for (int i = 0; i < MOTOR_COUNT; i++) r.motor[i] = out.motor[i];
    r.rcThrottle = rc::input().throttle;
    Angles angles = attitude.angles();
    r.rollDeg = angles.rollDeg;
    r.pitchDeg = angles.pitchDeg;
    r.batteryMv = battery::millivolts();
    r.armState = (uint8_t)modes.armState();
    r.flightMode = (uint8_t)modes.flightMode();
    r.auxBits = 0;
    for (int i = 0; i < RC_AUX_COUNT; i++) r.auxBits |= (uint8_t)((int)rc::input().aux[i] << (i * 2));
    blackbox::logLoop(r);
}

// ---------------------------------------------------------------------------------
// Slow tasks: each has its own interval, at most one runs per loop tick so the
// expensive ones never stack up in the same tick.

static void taskBattery(uint32_t)
{
    battery::update();
}

static void taskTelemetry(uint32_t)
{
    telemetry::update(attitude.angles(), modes.flightModeName());
}

static void taskEventLog(uint32_t)
{
    eventLog::flush();
}

static void taskStatusLed(uint32_t nowMs)
{
    statusLed::update(nowMs, modes.armState(), modes.armBlockFlags());
}

static void taskCli(uint32_t nowMs)
{
    cli::update(nowMs, modes.isArmed());
    if (params::revision() != appliedParamsRevision) applyParams();
}

static void taskPerf(uint32_t)
{
    perf::rollWindow();
    if (perf::overrunsInWindow() > 0) {
        eventLog::post(LogLevel::Warn, LogCode::LoopOverrun, (int32_t)perf::overrunsInWindow());
    }
    if (rateController.faults() != lastPidFaults) {
        lastPidFaults = rateController.faults();
        eventLog::post(LogLevel::Error, LogCode::PidFault, (int32_t)lastPidFaults);
    }

    static uint32_t lastRcFrames = 0;
    BbPerfRecord record;
    record.timeUs = board::micros();
    record.loopAvgUs = (uint16_t)perf::loop().avgUs;
    record.loopMaxUs = (uint16_t)perf::loop().maxUs;
    record.cpuLoadPermille = (uint16_t)perf::cpuLoadPermille();
    record.maxLatenessUs = (uint16_t)perf::maxLatenessUs();
    record.overruns = perf::overrunsTotal();
    record.droppedRecords = blackbox::droppedRecords();
    record.sdWriteMaxUs = perf::sdWriteMaxUs();
    record.freeHeap = board::freeHeap();
    record.rcFramesPerSecond = (uint16_t)(rc::goodFrames() - lastRcFrames);
    lastRcFrames = rc::goodFrames();
    blackbox::logPerf(record);
}

struct SlowTask {
    void (*run)(uint32_t nowMs);
    uint32_t intervalMs;
    uint32_t lastRunMs;
};

static SlowTask slowTasks[] = {
    { taskBattery, 20, 0 },
    { taskEventLog, 50, 0 },
    { taskStatusLed, 50, 0 },
    { taskCli, 50, 0 },
    { taskTelemetry, 100, 0 },
    { taskPerf, 1000, 0 },
};

static void runSlowTasks(uint32_t nowMs)
{
    // These two are cheap and need every tick to keep their buffers moving.
    console::flush();
    blackbox::service(nowMs);

    for (unsigned i = 0; i < sizeof(slowTasks) / sizeof(slowTasks[0]); i++) {
        SlowTask& task = slowTasks[i];
        if (nowMs - task.lastRunMs >= task.intervalMs) {
            task.lastRunMs = nowMs;
            task.run(nowMs);
            return;
        }
    }
}
