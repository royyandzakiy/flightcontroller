#include "system/event_log.h"

#include "board/board.h"
#include "mode_manager.h"
#include "system/blackbox.h"
#include "system/console.h"

namespace {

struct Event {
    uint32_t timeUs;
    LogLevel level;
    LogCode code;
    int32_t a;
    int32_t b;
};

// Text per code. Each may use up to two %ld for the event's a and b values.
const char* const TEXT[(int)LogCode::Count] = {
    "boot, reset reason code %ld",
    "last reset was the watchdog",
    "IMU found, WHO_AM_I 0x%lx",
    "IMU failed after %ld read errors",
    "gyro calibrated",
    "gyro calibration retry, quad moved",
    "accel calibrated",
    "armed",
    "arm blocked, flags 0x%lx",
    "disarmed, reason %ld:",
    "flight mode %ld (0 acro, 1 angle)",
    "FAILSAFE started",
    "failsafe recovered, link back",
    "battery state %ld, %ld mV",
    "RC link lost",
    "RC link restored",
    "SD card ready, next log %ld",
    "SD card error %ld",
    "blackbox started, log %ld",
    "blackbox stopped, %ld bytes",
    "blackbox file full",
    "loop overruns: %ld in the last second",
    "PID output was NaN, %ld total",
    "params loaded",
    "params invalid or missing, using defaults",
    "params saved",
    "CLI opened",
    "CLI closed",
    "motor test: motor %ld at %ld %%",
};

const char* const LEVEL_NAMES[] = { "ERROR", "WARN ", "INFO ", "DEBUG" };

const int QUEUE_SIZE = 16;
Event queue[QUEUE_SIZE];
int queueHead = 0;   // next write
int queueCount = 0;
uint32_t dropped = 0;
LogLevel consoleLevel = LogLevel::Info;

void printEvent(const Event& event)
{
    if ((int)event.level > (int)consoleLevel) return;

    unsigned long seconds = event.timeUs / 1000000UL;
    unsigned long millis = (event.timeUs / 1000UL) % 1000UL;
    console::printf("[%5lu.%03lu] %s ", seconds, millis, LEVEL_NAMES[(int)event.level]);
    console::printf(TEXT[(int)event.code], (long)event.a, (long)event.b);
    if (event.code == LogCode::Disarmed) {
        console::printf(" %s", ModeManager::disarmReasonName((DisarmReason)event.a));
    }
    console::print("\r\n");

    // Spell out the arm block reasons, the hex value alone is hard to read.
    if (event.code == LogCode::ArmBlocked) {
        for (int bit = 0; bit < ARM_BLOCK_FLAG_COUNT; bit++) {
            if (event.a & (1 << bit)) {
                console::printf("              - %s\r\n", ModeManager::armBlockFlagName(bit));
            }
        }
    }
}

}  // namespace

namespace eventLog {

void init()
{
    queueHead = 0;
    queueCount = 0;
}

void post(LogLevel level, LogCode code, int32_t a, int32_t b)
{
    if (queueCount == QUEUE_SIZE) {
        // Full: overwrite the oldest, recent events matter more.
        queueCount--;
        dropped++;
    }
    Event& event = queue[queueHead];
    event.timeUs = board::micros();
    event.level = level;
    event.code = code;
    event.a = a;
    event.b = b;
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    queueCount++;
}

void flush()
{
    // Two per call keeps the slow task short, the queue drains within a few calls.
    for (int i = 0; i < 2 && queueCount > 0; i++) {
        int tail = (queueHead - queueCount + QUEUE_SIZE) % QUEUE_SIZE;
        const Event& event = queue[tail];
        printEvent(event);
        blackbox::logEvent(event.timeUs, (uint8_t)event.level, (uint8_t)event.code, event.a, event.b);
        queueCount--;
    }
}

bool pending()
{
    return queueCount > 0;
}

void setConsoleLevel(LogLevel level)
{
    consoleLevel = level;
}

uint32_t droppedEvents()
{
    return dropped;
}

}  // namespace eventLog
