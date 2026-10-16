#include "bb_record.h"

#include "byte_io.h"
#include "fc_math.h"

static int16_t toI16(float value, float scale)
{
    if (!fcIsFinite(value)) return 0;
    return (int16_t)fcClampInt(fcRound(value * scale), -32768, 32767);
}

static uint16_t toU16(float value, float scale)
{
    if (!fcIsFinite(value)) return 0;
    return (uint16_t)fcClampInt(fcRound(value * scale), 0, 65535);
}

static uint32_t finish(const ByteWriter& writer, uint32_t expectedSize)
{
    if (writer.overflowed() || writer.length() != expectedSize) return 0;
    return writer.length();
}

uint32_t bbEncodeLoop(const BbLoopRecord& r, uint8_t* out, uint32_t capacity)
{
    ByteWriter w(out, capacity);
    w.u8(BB_TAG_LOOP);
    w.u32le(r.timeUs);
    w.u16le(r.loopUs);
    for (int i = 0; i < 3; i++) w.i16le(toI16(r.gyroRawDps[i], 10.0f));
    for (int i = 0; i < 3; i++) w.i16le(toI16(r.gyroDps[i], 10.0f));
    for (int i = 0; i < 3; i++) w.i16le(toI16(r.setpointDps[i], 10.0f));
    for (int i = 0; i < 3; i++) w.i16le(toI16(r.pidP[i], 10000.0f));
    for (int i = 0; i < 3; i++) w.i16le(toI16(r.pidI[i], 10000.0f));
    for (int i = 0; i < 3; i++) w.i16le(toI16(r.pidD[i], 10000.0f));
    for (int i = 0; i < 4; i++) w.u16le(toU16(r.motor[i], 1000.0f));
    for (int i = 0; i < 3; i++) w.i16le(toI16(r.rcStick[i], 1000.0f));
    w.i16le(toI16(r.rcThrottle, 1000.0f));
    w.i16le(toI16(r.rollDeg, 10.0f));
    w.i16le(toI16(r.pitchDeg, 10.0f));
    w.u16le(r.batteryMv);
    w.u8(r.armState);
    w.u8(r.flightMode);
    w.u8(r.auxBits);
    return finish(w, BB_LOOP_RECORD_SIZE);
}

uint32_t bbEncodeEvent(const BbEventRecord& r, uint8_t* out, uint32_t capacity)
{
    ByteWriter w(out, capacity);
    w.u8(BB_TAG_EVENT);
    w.u32le(r.timeUs);
    w.u8(r.level);
    w.u8(r.code);
    w.i32le(r.a);
    w.i32le(r.b);
    return finish(w, BB_EVENT_RECORD_SIZE);
}

uint32_t bbEncodePerf(const BbPerfRecord& r, uint8_t* out, uint32_t capacity)
{
    ByteWriter w(out, capacity);
    w.u8(BB_TAG_PERF);
    w.u32le(r.timeUs);
    w.u16le(r.loopAvgUs);
    w.u16le(r.loopMaxUs);
    w.u16le(r.cpuLoadPermille);
    w.u16le(r.maxLatenessUs);
    w.u32le(r.overruns);
    w.u32le(r.droppedRecords);
    w.u32le(r.sdWriteMaxUs);
    w.u32le(r.freeHeap);
    w.u16le(r.rcFramesPerSecond);
    return finish(w, BB_PERF_RECORD_SIZE);
}

bool bbDecodeLoop(const uint8_t* in, uint32_t length, BbLoopRecord& r)
{
    if (length < BB_LOOP_RECORD_SIZE || in[0] != BB_TAG_LOOP) return false;
    ByteReader rd(in + 1, BB_LOOP_RECORD_SIZE - 1);
    r.timeUs = rd.u32le();
    r.loopUs = rd.u16le();
    for (int i = 0; i < 3; i++) r.gyroRawDps[i] = rd.i16le() / 10.0f;
    for (int i = 0; i < 3; i++) r.gyroDps[i] = rd.i16le() / 10.0f;
    for (int i = 0; i < 3; i++) r.setpointDps[i] = rd.i16le() / 10.0f;
    for (int i = 0; i < 3; i++) r.pidP[i] = rd.i16le() / 10000.0f;
    for (int i = 0; i < 3; i++) r.pidI[i] = rd.i16le() / 10000.0f;
    for (int i = 0; i < 3; i++) r.pidD[i] = rd.i16le() / 10000.0f;
    for (int i = 0; i < 4; i++) r.motor[i] = rd.u16le() / 1000.0f;
    for (int i = 0; i < 3; i++) r.rcStick[i] = rd.i16le() / 1000.0f;
    r.rcThrottle = rd.i16le() / 1000.0f;
    r.rollDeg = rd.i16le() / 10.0f;
    r.pitchDeg = rd.i16le() / 10.0f;
    r.batteryMv = rd.u16le();
    r.armState = rd.u8();
    r.flightMode = rd.u8();
    r.auxBits = rd.u8();
    return !rd.overflowed();
}

bool bbDecodeEvent(const uint8_t* in, uint32_t length, BbEventRecord& r)
{
    if (length < BB_EVENT_RECORD_SIZE || in[0] != BB_TAG_EVENT) return false;
    ByteReader rd(in + 1, BB_EVENT_RECORD_SIZE - 1);
    r.timeUs = rd.u32le();
    r.level = rd.u8();
    r.code = rd.u8();
    r.a = rd.i32le();
    r.b = rd.i32le();
    return !rd.overflowed();
}
