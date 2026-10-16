#include <unity.h>
#include <string.h>

#include "byte_ring_buffer.h"
#include "byte_io.h"
#include "crc.h"
#include "loop_timer.h"
#include "text_format.h"

void setUp() {}
void tearDown() {}

static void test_ring_buffer_fifo_order()
{
    uint8_t storage[8];
    ByteRingBuffer ring;
    ring.init(storage, sizeof(storage));

    const uint8_t in[] = { 1, 2, 3, 4, 5 };
    TEST_ASSERT_TRUE(ring.push(in, 5));
    uint8_t out[5] = { 0 };
    TEST_ASSERT_EQUAL_UINT32(3, ring.pop(out, 3));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(in, out, 3);
    TEST_ASSERT_EQUAL_UINT32(2, ring.used());
}

static void test_ring_buffer_wraps_around()
{
    uint8_t storage[8];
    ByteRingBuffer ring;
    ring.init(storage, sizeof(storage));

    const uint8_t first[] = { 1, 2, 3, 4, 5, 6 };
    uint8_t out[8];
    ring.push(first, 6);
    ring.pop(out, 6);

    const uint8_t second[] = { 10, 11, 12, 13, 14, 15, 16, 17 };
    TEST_ASSERT_TRUE(ring.push(second, 8));
    TEST_ASSERT_EQUAL_UINT32(8, ring.pop(out, 8));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(second, out, 8);
}

static void test_ring_buffer_rejects_partial_push()
{
    uint8_t storage[4];
    ByteRingBuffer ring;
    ring.init(storage, sizeof(storage));

    const uint8_t data[] = { 1, 2, 3, 4, 5 };
    TEST_ASSERT_FALSE(ring.push(data, 5));
    TEST_ASSERT_EQUAL_UINT32(0, ring.used());
}

static void test_crc8_known_value()
{
    // CRC-8 DVB-S2 check value for "123456789".
    const uint8_t text[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    TEST_ASSERT_EQUAL_HEX8(0xBC, crc8DvbS2Buffer(text, sizeof(text)));
}

static void test_crc16_known_value()
{
    // CRC-16 CCITT-FALSE check value for "123456789".
    const uint8_t text[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16Ccitt(text, sizeof(text)));
}

static void test_byte_io_round_trip()
{
    uint8_t buffer[16];
    ByteWriter w(buffer, sizeof(buffer));
    w.u8(0xAB);
    w.u16le(0x1234);
    w.i16le(-2);
    w.u32le(0xDEADBEEF);
    w.i32le(-100000);
    TEST_ASSERT_FALSE(w.overflowed());
    TEST_ASSERT_EQUAL_HEX8(0x34, buffer[1]);  // little endian

    ByteReader r(buffer, w.length());
    TEST_ASSERT_EQUAL_HEX8(0xAB, r.u8());
    TEST_ASSERT_EQUAL_HEX16(0x1234, r.u16le());
    TEST_ASSERT_EQUAL_INT16(-2, r.i16le());
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, r.u32le());
    TEST_ASSERT_EQUAL_INT32(-100000, r.i32le());
    TEST_ASSERT_FALSE(r.overflowed());
    r.u8();
    TEST_ASSERT_TRUE(r.overflowed());
}

static void test_byte_writer_overflow()
{
    uint8_t buffer[3];
    ByteWriter w(buffer, sizeof(buffer));
    w.u16be(0x0102);
    w.u16be(0x0304);
    TEST_ASSERT_TRUE(w.overflowed());
    TEST_ASSERT_EQUAL_UINT32(2, w.length());
}

static void test_loop_timer_ticks_on_grid()
{
    LoopTimer timer;
    timer.init(1000, 0);
    TEST_ASSERT_FALSE(timer.isTickDue(999));
    TEST_ASSERT_TRUE(timer.isTickDue(1000));
    TEST_ASSERT_FALSE(timer.isTickDue(1500));
    // 200 us late, the next tick stays on the grid at 3000.
    TEST_ASSERT_TRUE(timer.isTickDue(2200));
    TEST_ASSERT_EQUAL_UINT32(200, timer.lastLatenessUs());
    TEST_ASSERT_FALSE(timer.isTickDue(2999));
    TEST_ASSERT_TRUE(timer.isTickDue(3000));
    TEST_ASSERT_EQUAL_UINT32(0, timer.overruns());
}

static void test_loop_timer_counts_overrun()
{
    LoopTimer timer;
    timer.init(1000, 0);
    TEST_ASSERT_TRUE(timer.isTickDue(3500));
    TEST_ASSERT_EQUAL_UINT32(1, timer.overruns());
    // Grid restarts from the late tick.
    TEST_ASSERT_FALSE(timer.isTickDue(4499));
    TEST_ASSERT_TRUE(timer.isTickDue(4500));
}

static void test_loop_timer_handles_wrap()
{
    LoopTimer timer;
    timer.init(1000, 0xFFFFFE00u);
    TEST_ASSERT_FALSE(timer.isTickDue(0xFFFFFF00u));
    TEST_ASSERT_TRUE(timer.isTickDue(0x000001E8u));
}

static void test_format_float()
{
    char text[16];
    fcFormatFloat(text, sizeof(text), 1.5f, 3);
    TEST_ASSERT_EQUAL_STRING("1.500", text);
    fcFormatFloat(text, sizeof(text), -0.0025f, 4);
    TEST_ASSERT_EQUAL_STRING("-0.0025", text);
    fcFormatFloat(text, sizeof(text), 42.0f, 0);
    TEST_ASSERT_EQUAL_STRING("42", text);
    fcFormatFloat(text, sizeof(text), 0.99996f, 4);
    TEST_ASSERT_EQUAL_STRING("1.0000", text);
}

static void test_parse_float()
{
    float value = 0.0f;
    TEST_ASSERT_TRUE(fcParseFloat("0.0025", value));
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0025f, value);
    TEST_ASSERT_TRUE(fcParseFloat("-12", value));
    TEST_ASSERT_EQUAL_FLOAT(-12.0f, value);
    TEST_ASSERT_TRUE(fcParseFloat("+.5", value));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, value);
    TEST_ASSERT_TRUE(fcParseFloat("2e-5", value));
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.00002f, value);
    TEST_ASSERT_TRUE(fcParseFloat("1.5E2", value));
    TEST_ASSERT_EQUAL_FLOAT(150.0f, value);

    TEST_ASSERT_FALSE(fcParseFloat("", value));
    TEST_ASSERT_FALSE(fcParseFloat("-", value));
    TEST_ASSERT_FALSE(fcParseFloat("1.2.3", value));
    TEST_ASSERT_FALSE(fcParseFloat("12abc", value));
    TEST_ASSERT_FALSE(fcParseFloat("1e", value));
    TEST_ASSERT_FALSE(fcParseFloat("1e99", value));
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_ring_buffer_fifo_order);
    RUN_TEST(test_ring_buffer_wraps_around);
    RUN_TEST(test_ring_buffer_rejects_partial_push);
    RUN_TEST(test_crc8_known_value);
    RUN_TEST(test_crc16_known_value);
    RUN_TEST(test_byte_io_round_trip);
    RUN_TEST(test_byte_writer_overflow);
    RUN_TEST(test_loop_timer_ticks_on_grid);
    RUN_TEST(test_loop_timer_counts_overrun);
    RUN_TEST(test_loop_timer_handles_wrap);
    RUN_TEST(test_format_float);
    RUN_TEST(test_parse_float);
    return UNITY_END();
}
