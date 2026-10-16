#include "system/blackbox.h"

#include <SdFat.h>
#include <stdio.h>
#include <string.h>

#include "board/board.h"
#include "byte_ring_buffer.h"
#include "config.h"
#include "system/event_log.h"
#include "system/params.h"
#include "system/perf.h"
#include "text_format.h"

namespace {

enum class State : uint8_t {
    NoCard,     // card missing or failed, blackbox off
    Ready,      // next file created and pre-allocated, waiting for arm
    Logging,
    Stopping,   // disarmed, flush the rest and close
    Error,
};

const uint32_t BLOCK_SIZE = 512;
const uint32_t MAX_LOG_INDEX = 9999;

SdFat32 sd;
File32 file;
State state = State::NoCard;
ByteRingBuffer ring;
uint8_t block[BLOCK_SIZE];
char name[16] = "";
uint32_t logIndex = 0;
uint32_t written = 0;
uint32_t dropped = 0;
uint8_t rateDivider = 1;
uint8_t loopCounter = 0;
uint32_t lastSyncMs = 0;
bool fullReported = false;

void push(const uint8_t* data, uint32_t length)
{
    if (state != State::Logging) return;
    if (!ring.push(data, length)) dropped++;
}

void pushText(const char* text)
{
    push(reinterpret_cast<const uint8_t*>(text), (uint32_t)strlen(text));
}

void fail(int32_t code)
{
    state = State::Error;
    file.close();
    eventLog::post(LogLevel::Error, LogCode::SdError, code);
}

bool prepareNextFile()
{
    while (logIndex <= MAX_LOG_INDEX) {
        snprintf(name, sizeof(name), "LOG%05lu.BBL", (unsigned long)logIndex);
        if (!sd.exists(name)) break;
        logIndex++;
    }
    if (logIndex > MAX_LOG_INDEX) return false;

    if (!file.open(name, O_RDWR | O_CREAT | O_TRUNC)) return false;
    // A contiguous file avoids the FAT lookups that cause most SD write stalls.
    // If the card has no contiguous space left, logging still works, just with more jitter.
    file.preAllocate(BLACKBOX_PREALLOCATE_BYTES);
    written = 0;
    state = State::Ready;
    return true;
}

void writeHeader(uint32_t nowMs)
{
    char line[64];
    char number[20];
    snprintf(line, sizeof(line), "FCBB %u\n", (unsigned)BB_FORMAT_VERSION);
    pushText(line);
    snprintf(line, sizeof(line), "board=%s\n", board::info().name);
    pushText(line);
    snprintf(line, sizeof(line), "loop_hz=%u\n", (unsigned)board::info().loopHz);
    pushText(line);
    snprintf(line, sizeof(line), "log_div=%u\n", (unsigned)rateDivider);
    pushText(line);
    snprintf(line, sizeof(line), "start_ms=%lu\n", (unsigned long)nowMs);
    pushText(line);
    snprintf(line, sizeof(line), "record_sizes=L:%lu,E:%lu,P:%lu\n",
             (unsigned long)BB_LOOP_RECORD_SIZE, (unsigned long)BB_EVENT_RECORD_SIZE,
             (unsigned long)BB_PERF_RECORD_SIZE);
    pushText(line);
    for (int i = 0; i < params::count(); i++) {
        const ParamDef& param = params::def(i);
        fcFormatFloat(number, sizeof(number), *param.value, 6);
        snprintf(line, sizeof(line), "param.%s=%s\n", param.name, number);
        pushText(line);
    }
    pushText("end\n");
}

void writeBlock(uint32_t length)
{
    uint32_t count = ring.pop(block, length);
    uint32_t startUs = board::micros();
    size_t result = file.write(block, count);
    perf::noteSdWrite(board::micros() - startUs);
    if (result != count) {
        fail(1);
        return;
    }
    written += count;
}

void closeFile()
{
    while (ring.used() > 0 && state != State::Error) {
        writeBlock(BLOCK_SIZE);
        board::watchdogKick();
    }
    if (state == State::Error) return;
    // Drop the unused pre-allocated space, the file ends where the data ends.
    file.truncate();
    file.close();
    eventLog::post(LogLevel::Info, LogCode::BlackboxStopped, (int32_t)written);
    board::watchdogKick();
    logIndex++;
    if (!prepareNextFile()) fail(2);
}

}  // namespace

namespace blackbox {

void init()
{
    uint32_t size = 0;
    uint8_t* storage = board::blackboxBuffer(size);
    ring.init(storage, size);

    SdSpiConfig config(board::sdCsPin(), DEDICATED_SPI, SD_SCK_MHZ(16), &board::sdSpi());
    if (!sd.begin(config)) {
        state = State::NoCard;
        eventLog::post(LogLevel::Warn, LogCode::SdError, 0);
        return;
    }
    logIndex = 1;
    if (!prepareNextFile()) {
        fail(2);
        return;
    }
    eventLog::post(LogLevel::Info, LogCode::SdReady, (int32_t)logIndex);
}

void setRateDivider(uint8_t divider)
{
    rateDivider = divider > 0 ? divider : 1;
}

void start(uint32_t nowMs)
{
    if (state != State::Ready) return;
    ring.clear();
    state = State::Logging;
    loopCounter = 0;
    lastSyncMs = nowMs;
    fullReported = false;
    // About 40 short lines: one slow tick at arm, which is fine once per flight.
    writeHeader(nowMs);
    eventLog::post(LogLevel::Info, LogCode::BlackboxStarted, (int32_t)logIndex);
}

void requestStop()
{
    if (state == State::Logging) state = State::Stopping;
}

void logLoop(const BbLoopRecord& record)
{
    if (state != State::Logging) return;
    loopCounter++;
    if (loopCounter < rateDivider) return;
    loopCounter = 0;

    uint8_t buffer[BB_LOOP_RECORD_SIZE];
    uint32_t length = bbEncodeLoop(record, buffer, sizeof(buffer));
    if (length > 0) push(buffer, length);
}

void logEvent(uint32_t timeUs, uint8_t level, uint8_t code, int32_t a, int32_t b)
{
    BbEventRecord record;
    record.timeUs = timeUs;
    record.level = level;
    record.code = code;
    record.a = a;
    record.b = b;
    uint8_t buffer[BB_EVENT_RECORD_SIZE];
    uint32_t length = bbEncodeEvent(record, buffer, sizeof(buffer));
    if (length > 0) push(buffer, length);
}

void logPerf(const BbPerfRecord& record)
{
    uint8_t buffer[BB_PERF_RECORD_SIZE];
    uint32_t length = bbEncodePerf(record, buffer, sizeof(buffer));
    if (length > 0) push(buffer, length);
}

void service(uint32_t nowMs)
{
    if (state == State::Stopping) {
        // Blocking, but only ever runs disarmed with the motors already stopped.
        closeFile();
        if (state == State::Stopping) state = State::Ready;
        return;
    }
    if (state != State::Logging) return;

    if (written + BLOCK_SIZE > BLACKBOX_PREALLOCATE_BYTES) {
        if (!fullReported) {
            eventLog::post(LogLevel::Warn, LogCode::BlackboxFull);
            fullReported = true;
        }
        ring.clear();
        return;
    }

    if (ring.used() >= BLOCK_SIZE) {
        writeBlock(BLOCK_SIZE);
    } else if (nowMs - lastSyncMs >= BLACKBOX_SYNC_INTERVAL_MS) {
        // Updates the directory entry, so a power cut keeps everything up to here.
        lastSyncMs = nowMs;
        if (!file.sync()) fail(3);
    }
}

bool available()
{
    return state == State::Ready || state == State::Logging || state == State::Stopping;
}

bool logging()
{
    return state == State::Logging;
}

const char* fileName()
{
    return name;
}

uint32_t bytesWritten()
{
    return written;
}

uint32_t droppedRecords()
{
    return dropped;
}

uint32_t bufferUsed()
{
    return ring.used();
}

uint32_t bufferSize()
{
    return ring.size();
}

}  // namespace blackbox
