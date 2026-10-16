#include <unity.h>
#include <string.h>

#include "bb_record.h"

void setUp() {}
void tearDown() {}

static BbLoopRecord sampleLoopRecord()
{
    BbLoopRecord r;
    memset(&r, 0, sizeof(r));
    r.timeUs = 123456789;
    r.loopUs = 412;
    for (int i = 0; i < 3; i++) {
        r.gyroRawDps[i] = 100.5f * (i + 1);
        r.gyroDps[i] = -50.2f * (i + 1);
        r.setpointDps[i] = 200.0f;
        r.pidP[i] = 0.1234f;
        r.pidI[i] = -0.05f;
        r.pidD[i] = 0.0021f;
        r.rcStick[i] = -0.5f;
    }
    for (int i = 0; i < 4; i++) r.motor[i] = 0.25f * i;
    r.rcThrottle = 0.75f;
    r.rollDeg = -12.3f;
    r.pitchDeg = 4.5f;
    r.batteryMv = 15800;
    r.armState = 1;
    r.flightMode = 0;
    r.auxBits = 0x26;
    return r;
}

static void test_loop_record_size_and_round_trip()
{
    BbLoopRecord in = sampleLoopRecord();
    uint8_t buffer[128];
    TEST_ASSERT_EQUAL_UINT32(BB_LOOP_RECORD_SIZE, bbEncodeLoop(in, buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL_HEX8(BB_TAG_LOOP, buffer[0]);

    BbLoopRecord out;
    TEST_ASSERT_TRUE(bbDecodeLoop(buffer, BB_LOOP_RECORD_SIZE, out));
    TEST_ASSERT_EQUAL_UINT32(in.timeUs, out.timeUs);
    TEST_ASSERT_EQUAL_UINT16(in.loopUs, out.loopUs);
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.05f, in.gyroRawDps[i], out.gyroRawDps[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.05f, in.gyroDps[i], out.gyroDps[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, in.pidP[i], out.pidP[i]);
    }
    for (int i = 0; i < 4; i++) TEST_ASSERT_FLOAT_WITHIN(0.001f, in.motor[i], out.motor[i]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, in.rollDeg, out.rollDeg);
    TEST_ASSERT_EQUAL_UINT16(in.batteryMv, out.batteryMv);
    TEST_ASSERT_EQUAL_HEX8(in.auxBits, out.auxBits);
}

static void test_loop_record_saturates_out_of_range()
{
    BbLoopRecord in = sampleLoopRecord();
    in.gyroRawDps[0] = 99999.0f;
    in.pidD[0] = -5.0f;
    uint8_t buffer[128];
    bbEncodeLoop(in, buffer, sizeof(buffer));
    BbLoopRecord out;
    bbDecodeLoop(buffer, BB_LOOP_RECORD_SIZE, out);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 3276.7f, out.gyroRawDps[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -3.2768f, out.pidD[0]);
}

static void test_encode_fails_when_buffer_small()
{
    BbLoopRecord in = sampleLoopRecord();
    uint8_t buffer[20];
    TEST_ASSERT_EQUAL_UINT32(0, bbEncodeLoop(in, buffer, sizeof(buffer)));
}

static void test_event_round_trip()
{
    BbEventRecord in = { 42, 2, 7, -1, 0x7FFFFFFF };
    uint8_t buffer[32];
    TEST_ASSERT_EQUAL_UINT32(BB_EVENT_RECORD_SIZE, bbEncodeEvent(in, buffer, sizeof(buffer)));
    BbEventRecord out;
    TEST_ASSERT_TRUE(bbDecodeEvent(buffer, BB_EVENT_RECORD_SIZE, out));
    TEST_ASSERT_EQUAL_UINT32(42, out.timeUs);
    TEST_ASSERT_EQUAL_UINT8(7, out.code);
    TEST_ASSERT_EQUAL_INT32(-1, out.a);
    TEST_ASSERT_EQUAL_INT32(0x7FFFFFFF, out.b);
}

static void test_perf_record_size()
{
    BbPerfRecord in;
    memset(&in, 0, sizeof(in));
    uint8_t buffer[64];
    TEST_ASSERT_EQUAL_UINT32(BB_PERF_RECORD_SIZE, bbEncodePerf(in, buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL_HEX8(BB_TAG_PERF, buffer[0]);
}

static void test_decode_rejects_wrong_tag()
{
    uint8_t buffer[BB_LOOP_RECORD_SIZE];
    memset(buffer, 0, sizeof(buffer));
    BbLoopRecord out;
    TEST_ASSERT_FALSE(bbDecodeLoop(buffer, sizeof(buffer), out));
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_loop_record_size_and_round_trip);
    RUN_TEST(test_loop_record_saturates_out_of_range);
    RUN_TEST(test_encode_fails_when_buffer_small);
    RUN_TEST(test_event_round_trip);
    RUN_TEST(test_perf_record_size);
    RUN_TEST(test_decode_rejects_wrong_tag);
    return UNITY_END();
}
