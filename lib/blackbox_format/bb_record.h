// Binary blackbox records. Each record is a 1-byte tag followed by a fixed-size,
// little-endian body of scaled integers. tools/bb_decode.py mirrors this layout.
//
// A log file is a text header ("FCBB 1" ... "end") followed by records back to back.
// A zero byte where a tag is expected marks the end of the data.
#pragma once

#include <stdint.h>

static const uint8_t BB_FORMAT_VERSION = 1;

static const uint8_t BB_TAG_LOOP = 'L';
static const uint8_t BB_TAG_EVENT = 'E';
static const uint8_t BB_TAG_PERF = 'P';

// Encoded sizes including the tag byte.
static const uint32_t BB_LOOP_RECORD_SIZE = 68;
static const uint32_t BB_EVENT_RECORD_SIZE = 15;
static const uint32_t BB_PERF_RECORD_SIZE = 31;

struct BbLoopRecord {
    uint32_t timeUs;
    uint16_t loopUs;          // time the previous loop body took
    float gyroRawDps[3];      // stored as dps * 10
    float gyroDps[3];         // stored as dps * 10
    float setpointDps[3];     // stored as dps * 10
    float pidP[3];            // stored as value * 10000
    float pidI[3];
    float pidD[3];
    float motor[4];           // 0..1, stored as value * 1000
    float rcStick[3];         // -1..1, stored as value * 1000
    float rcThrottle;         // 0..1, stored as value * 1000
    float rollDeg;            // stored as deg * 10
    float pitchDeg;
    uint16_t batteryMv;
    uint8_t armState;
    uint8_t flightMode;
    uint8_t auxBits;          // 2 bits per aux channel, 0 low, 1 mid, 2 high
};

struct BbEventRecord {
    uint32_t timeUs;
    uint8_t level;
    uint8_t code;
    int32_t a;
    int32_t b;
};

struct BbPerfRecord {
    uint32_t timeUs;
    uint16_t loopAvgUs;
    uint16_t loopMaxUs;
    uint16_t cpuLoadPermille;
    uint16_t maxLatenessUs;
    uint32_t overruns;          // total since boot
    uint32_t droppedRecords;    // blackbox records lost to a full buffer
    uint32_t sdWriteMaxUs;      // slowest SD block write in the last window
    uint32_t freeHeap;
    uint16_t rcFramesPerSecond;
};

// Each encode returns the bytes written, or 0 if capacity is too small.
uint32_t bbEncodeLoop(const BbLoopRecord& record, uint8_t* out, uint32_t capacity);
uint32_t bbEncodeEvent(const BbEventRecord& record, uint8_t* out, uint32_t capacity);
uint32_t bbEncodePerf(const BbPerfRecord& record, uint8_t* out, uint32_t capacity);

// Decoders for tests. Expect the tag byte at in[0], return false on a wrong tag or size.
bool bbDecodeLoop(const uint8_t* in, uint32_t length, BbLoopRecord& record);
bool bbDecodeEvent(const uint8_t* in, uint32_t length, BbEventRecord& record);
