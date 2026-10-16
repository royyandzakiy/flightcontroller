#include "system/cli.h"

#include <stdlib.h>
#include <string.h>

#include "board/board.h"
#include "config.h"
#include "drivers/battery.h"
#include "drivers/imu_mpu6050.h"
#include "drivers/rc_crsf.h"
#include "system/blackbox.h"
#include "system/console.h"
#include "system/event_log.h"
#include "system/params.h"
#include "system/perf.h"
#include "text_format.h"

namespace {

const uint32_t LINE_SIZE = 64;
const uint32_t MAX_BYTES_PER_UPDATE = 32;
const int MAX_ARGS = 4;

const ModeManager* modeSource = NULL;
const AttitudeEstimator* attitudeSource = NULL;

char line[LINE_SIZE];
uint32_t lineLength = 0;
bool lineOverflow = false;
bool sessionOpen = false;
uint32_t lastActivityMs = 0;

bool motorTestOn = false;
int motorTestMotor = 0;
float motorTestCommand = 0.0f;
uint32_t motorTestStartMs = 0;

// Formats a float into one of two rotating buffers, so two can share one printf.
const char* f(float value, uint8_t decimals)
{
    static char buffers[2][20];
    static int next = 0;
    char* out = buffers[next];
    next = 1 - next;
    fcFormatFloat(out, sizeof(buffers[0]), value, decimals);
    return out;
}

void openSession(uint32_t nowMs)
{
    if (!sessionOpen) {
        sessionOpen = true;
        eventLog::post(LogLevel::Info, LogCode::CliOpened);
    }
    lastActivityMs = nowMs;
}

void closeSession()
{
    if (sessionOpen) {
        sessionOpen = false;
        motorTestOn = false;
        eventLog::post(LogLevel::Info, LogCode::CliClosed);
    }
}

void printHelp()
{
    console::print(
        "commands:\r\n"
        "  status                      overall state, arm block reasons\r\n"
        "  perf                        loop timing per stage\r\n"
        "  get <name> | set <name> <v> read or change a parameter\r\n"
        "  dump                        all parameters\r\n"
        "  save | defaults             store parameters, or reset them\r\n"
        "  calib accel                 level calibration, quad flat and still\r\n"
        "  motor <1-4> <0-100> props-off   spin one motor for 2 s\r\n"
        "  motor stop\r\n"
        "  bb                          blackbox state\r\n"
        "  log level <0-3>             console log level (error..debug)\r\n"
        "  reboot | debug hang         restart, or hang to test the watchdog\r\n"
        "  exit                        close the session, allows arming\r\n");
}

void printStatus()
{
    const ModeManager& modes = *modeSource;
    Angles angles = attitudeSource->angles();

    console::printf("board %s, up %lu s, reset: %s\r\n", board::info().name,
                    (unsigned long)(board::millis() / 1000), board::resetReason());
    console::printf("state %s, mode %s\r\n",
                    modes.armState() == ArmState::Disarmed ? "DISARMED" : "ARMED",
                    modes.flightModeName());

    uint16_t flags = modes.armBlockFlags();
    console::printf("arm blocked by: %s", flags == 0 ? "nothing, ready" : "");
    for (int bit = 0; bit < ARM_BLOCK_FLAG_COUNT; bit++) {
        if (flags & (1 << bit)) console::printf("%s ", ModeManager::armBlockFlagName(bit));
    }
    console::print("\r\n");

    const CrsfLinkStats& link = rc::linkStats();
    console::printf("rc link %s, LQ %u%%, RSSI -%u dBm, frames %lu ok / %lu bad\r\n",
                    rc::linkOk() ? "ok" : "LOST", link.uplinkLq, link.uplinkRssi1,
                    (unsigned long)rc::goodFrames(), (unsigned long)rc::badFrames());
    console::print("rc us:");
    for (int ch = 0; ch < 8; ch++) console::printf(" %u", rc::channelUs(ch));
    console::print("\r\n");

    console::printf("attitude roll %s pitch %s deg\r\n", f(angles.rollDeg, 1), f(angles.pitchDeg, 1));
    console::printf("imu 0x%02x %s, gyro %s, %lu read errors\r\n", imu::whoAmI(),
                    imu::healthy() ? "ok" : "FAILED", imu::gyroCalibrated() ? "calibrated" : "NOT calibrated",
                    (unsigned long)imu::errorCount());
    console::printf("battery %s V, %u cells, %s\r\n", f(battery::volts(), 2), battery::cells(),
                    battery::stateName(battery::state()));
    console::printf("blackbox %s %s\r\n",
                    blackbox::logging() ? "logging" : (blackbox::available() ? "ready" : "off"),
                    blackbox::fileName());
    console::printf("free heap %lu, console dropped %lu bytes, log dropped %lu events\r\n",
                    (unsigned long)board::freeHeap(), (unsigned long)console::droppedBytes(),
                    (unsigned long)eventLog::droppedEvents());
}

void printPerf()
{
    console::print("stage        min   avg   max  (us, last second)\r\n");
    for (int i = 0; i < (int)perf::Stage::Count; i++) {
        const perf::Stats& s = perf::stage((perf::Stage)i);
        console::printf("  %-9s %5lu %5lu %5lu\r\n", perf::stageName((perf::Stage)i),
                        (unsigned long)s.minUs, (unsigned long)s.avgUs, (unsigned long)s.maxUs);
    }
    const perf::Stats& loop = perf::loop();
    console::printf("  %-9s %5lu %5lu %5lu   period %lu\r\n", "loop", (unsigned long)loop.minUs,
                    (unsigned long)loop.avgUs, (unsigned long)loop.maxUs,
                    (unsigned long)(1000000UL / board::info().loopHz));
    console::printf("cpu load %s %%, max lateness %lu us, overruns %lu (total %lu)\r\n",
                    f(perf::cpuLoadPermille() / 10.0f, 1), (unsigned long)perf::maxLatenessUs(),
                    (unsigned long)perf::overrunsInWindow(), (unsigned long)perf::overrunsTotal());
    console::printf("sd write max %lu us, blackbox buffer %lu/%lu, dropped %lu records\r\n",
                    (unsigned long)perf::sdWriteMaxUs(), (unsigned long)blackbox::bufferUsed(),
                    (unsigned long)blackbox::bufferSize(), (unsigned long)blackbox::droppedRecords());
}

void printParam(const ParamDef& param)
{
    console::printf("set %s %s\r\n", param.name, f(*param.value, 6));
}

void commandSet(int argc, char** argv)
{
    if (argc != 3) {
        console::print("usage: set <name> <value>\r\n");
        return;
    }
    const ParamDef* param = params::find(argv[1]);
    if (param == NULL) {
        console::printf("unknown parameter %s\r\n", argv[1]);
        return;
    }
    float value = 0.0f;
    if (!fcParseFloat(argv[2], value) || !params::set(argv[1], value)) {
        console::printf("invalid value, %s allows %s .. %s\r\n", param->name,
                        f(param->minValue, 6), f(param->maxValue, 6));
        return;
    }
    printParam(*param);
}

void commandCalibAccel()
{
    console::print("calibrating, keep the quad level and still...\r\n");
    console::flushBlocking();
    float offset[AXIS_COUNT];
    if (!imu::calibrateAccel(offset)) {
        console::print("failed: moved, not level, or IMU errors\r\n");
        return;
    }
    ParamValues& values = params::edit();
    values.accelOffsetX = offset[0];
    values.accelOffsetY = offset[1];
    values.accelOffsetZ = offset[2];
    params::markChanged();
    eventLog::post(LogLevel::Info, LogCode::AccelCalibrated);
    console::printf("offsets x %s y %s", f(offset[0], 4), f(offset[1], 4));
    console::printf(" z %s g, type save to keep them\r\n", f(offset[2], 4));
}

void commandMotor(int argc, char** argv, uint32_t nowMs)
{
    if (argc == 2 && strcmp(argv[1], "stop") == 0) {
        motorTestOn = false;
        console::print("motors stopped\r\n");
        return;
    }
    if (argc != 4 || strcmp(argv[3], "props-off") != 0) {
        console::print("usage: motor <1-4> <0-100> props-off   (remove the props first)\r\n");
        return;
    }
    int motor = atoi(argv[1]);
    int percent = atoi(argv[2]);
    if (motor < 1 || motor > 4 || percent < 0 || percent > 100) {
        console::print("motor 1-4, percent 0-100\r\n");
        return;
    }
    motorTestOn = true;
    motorTestMotor = motor - 1;
    motorTestCommand = percent / 100.0f;
    motorTestStartMs = nowMs;
    eventLog::post(LogLevel::Warn, LogCode::MotorTest, motor, percent);
    console::printf("motor %d at %d %% for 2 s\r\n", motor, percent);
}

void runCommand(char* text, uint32_t nowMs)
{
    char* argv[MAX_ARGS];
    int argc = 0;
    char* token = strtok(text, " \t");
    while (token != NULL && argc < MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(NULL, " \t");
    }
    if (argc == 0) return;

    const char* cmd = argv[0];
    if (strcmp(cmd, "help") == 0) {
        printHelp();
    } else if (strcmp(cmd, "status") == 0) {
        printStatus();
    } else if (strcmp(cmd, "perf") == 0) {
        printPerf();
    } else if (strcmp(cmd, "get") == 0 && argc == 2) {
        const ParamDef* param = params::find(argv[1]);
        if (param) printParam(*param);
        else console::printf("unknown parameter %s\r\n", argv[1]);
    } else if (strcmp(cmd, "set") == 0) {
        commandSet(argc, argv);
    } else if (strcmp(cmd, "dump") == 0) {
        for (int i = 0; i < params::count(); i++) printParam(params::def(i));
    } else if (strcmp(cmd, "save") == 0) {
        console::print(params::save() ? "saved\r\n" : "save FAILED\r\n");
    } else if (strcmp(cmd, "defaults") == 0) {
        params::resetToDefaults();
        console::print("defaults restored, type save to keep them\r\n");
    } else if (strcmp(cmd, "calib") == 0 && argc == 2 && strcmp(argv[1], "accel") == 0) {
        commandCalibAccel();
    } else if (strcmp(cmd, "motor") == 0) {
        commandMotor(argc, argv, nowMs);
    } else if (strcmp(cmd, "bb") == 0) {
        console::printf("blackbox %s, file %s, %lu bytes, dropped %lu records\r\n",
                        blackbox::logging() ? "logging" : (blackbox::available() ? "ready" : "off"),
                        blackbox::fileName(), (unsigned long)blackbox::bytesWritten(),
                        (unsigned long)blackbox::droppedRecords());
    } else if (strcmp(cmd, "log") == 0 && argc == 3 && strcmp(argv[1], "level") == 0) {
        if (!params::set("log_level", (float)atoi(argv[2]))) console::print("level 0-3\r\n");
    } else if (strcmp(cmd, "reboot") == 0) {
        console::print("rebooting\r\n");
        console::flushBlocking();
        board::reboot();
    } else if (strcmp(cmd, "debug") == 0 && argc == 2 && strcmp(argv[1], "hang") == 0) {
        console::print("hanging, the watchdog should reset the board\r\n");
        console::flushBlocking();
        while (true) {
        }
    } else if (strcmp(cmd, "exit") == 0) {
        console::print("bye\r\n");
        closeSession();
    } else {
        console::printf("unknown command %s, try help\r\n", cmd);
    }
}

}  // namespace

namespace cli {

void init(const ModeManager& modes, const AttitudeEstimator& attitude)
{
    modeSource = &modes;
    attitudeSource = &attitude;
}

void update(uint32_t nowMs, bool armed)
{
    Stream& serial = board::debugSerial();

    if (armed) {
        // No commands in flight: drop input and end any session.
        while (serial.available() > 0) serial.read();
        lineLength = 0;
        closeSession();
        return;
    }

    if (sessionOpen && nowMs - lastActivityMs > CLI_SESSION_TIMEOUT_MS) {
        console::print("CLI timeout\r\n");
        closeSession();
    }

    for (uint32_t i = 0; i < MAX_BYTES_PER_UPDATE && serial.available() > 0; i++) {
        int c = serial.read();
        if (c < 0) break;
        if (c == '\r' || c == '\n') {
            if (lineOverflow) {
                console::print("line too long\r\n");
            } else if (lineLength > 0) {
                line[lineLength] = '\0';
                openSession(nowMs);
                console::setBlocking(true);
                runCommand(line, nowMs);
                console::print("fc> ");
                console::setBlocking(false);
            }
            lineLength = 0;
            lineOverflow = false;
        } else if (lineLength < LINE_SIZE - 1) {
            line[lineLength++] = (char)c;
        } else {
            lineOverflow = true;
        }
    }
}

bool active()
{
    return sessionOpen;
}

bool motorTestActive(uint32_t nowMs)
{
    if (motorTestOn && nowMs - motorTestStartMs >= MOTOR_TEST_TIMEOUT_MS) motorTestOn = false;
    return motorTestOn;
}

int motorTestIndex()
{
    return motorTestMotor;
}

float motorTestValue()
{
    return motorTestCommand;
}

void stopMotorTest()
{
    motorTestOn = false;
}

}  // namespace cli
