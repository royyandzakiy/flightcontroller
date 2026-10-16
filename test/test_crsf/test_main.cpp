#include <unity.h>
#include <string.h>

#include "crc.h"
#include "crsf_parser.h"
#include "crsf_telemetry.h"

void setUp() {}
void tearDown() {}

// Packs 16 channel values into a complete RC_CHANNELS_PACKED frame, returns its length.
static uint8_t buildRcFrame(const uint16_t channels[16], uint8_t* frame)
{
    frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame[1] = CRSF_RC_PAYLOAD_SIZE + 2;
    frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    uint8_t* payload = &frame[3];
    memset(payload, 0, CRSF_RC_PAYLOAD_SIZE);

    uint32_t bitPosition = 0;
    for (int ch = 0; ch < 16; ch++) {
        for (int bit = 0; bit < 11; bit++) {
            if (channels[ch] & (1u << bit)) {
                payload[bitPosition / 8] |= (uint8_t)(1u << (bitPosition % 8));
            }
            bitPosition++;
        }
    }
    frame[3 + CRSF_RC_PAYLOAD_SIZE] = crc8DvbS2Buffer(&frame[2], CRSF_RC_PAYLOAD_SIZE + 1);
    return CRSF_RC_PAYLOAD_SIZE + 4;
}

static int feedAll(CrsfParser& parser, const uint8_t* data, uint32_t length)
{
    int frames = 0;
    for (uint32_t i = 0; i < length; i++) {
        if (parser.feed(data[i])) frames++;
    }
    return frames;
}

static void fillTestChannels(uint16_t channels[16])
{
    for (int ch = 0; ch < 16; ch++) channels[ch] = (uint16_t)(172 + ch * 100);
}

static void test_parses_rc_frame_and_unpacks_channels()
{
    uint16_t channels[16];
    fillTestChannels(channels);
    uint8_t frame[64];
    uint8_t length = buildRcFrame(channels, frame);

    CrsfParser parser;
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame, length));
    TEST_ASSERT_EQUAL_HEX8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, parser.frameType());
    TEST_ASSERT_EQUAL_UINT8(CRSF_RC_PAYLOAD_SIZE, parser.payloadLength());

    uint16_t decoded[16];
    crsfUnpackChannels(parser.payload(), decoded);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(channels, decoded, 16);
}

static void test_frame_split_across_feeds()
{
    uint16_t channels[16];
    fillTestChannels(channels);
    uint8_t frame[64];
    uint8_t length = buildRcFrame(channels, frame);

    CrsfParser parser;
    TEST_ASSERT_EQUAL_INT(0, feedAll(parser, frame, 10));
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame + 10, length - 10));
}

static void test_rejects_bad_crc()
{
    uint16_t channels[16];
    fillTestChannels(channels);
    uint8_t frame[64];
    uint8_t length = buildRcFrame(channels, frame);
    frame[5] ^= 0x01;

    CrsfParser parser;
    TEST_ASSERT_EQUAL_INT(0, feedAll(parser, frame, length));
    TEST_ASSERT_EQUAL_UINT32(1, parser.badCrcFrames());
}

static void test_resyncs_after_garbage()
{
    uint16_t channels[16];
    fillTestChannels(channels);
    uint8_t frame[64];
    uint8_t length = buildRcFrame(channels, frame);

    const uint8_t garbage[] = { 0x00, 0xFF, 0x13, 0x37, 0xC8, 0xFF, 0x42 };
    CrsfParser parser;
    feedAll(parser, garbage, sizeof(garbage));
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame, length));
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame, length));
}

static void test_rejects_bad_length()
{
    const uint8_t data[] = { 0xC8, 0x01, 0xC8, 0x50 };
    CrsfParser parser;
    TEST_ASSERT_EQUAL_INT(0, feedAll(parser, data, sizeof(data)));
    TEST_ASSERT_EQUAL_UINT32(2, parser.badLengthFrames());
}

static void test_channel_to_us()
{
    TEST_ASSERT_EQUAL_UINT16(1500, crsfChannelToUs(992));
    TEST_ASSERT_EQUAL_UINT16(988, crsfChannelToUs(172));
    TEST_ASSERT_UINT16_WITHIN(1, 2012, crsfChannelToUs(1811));
}

static void test_link_stats_decode()
{
    const uint8_t payload[10] = { 70, 72, 100, (uint8_t)-5, 1, 4, 3, 60, 98, 8 };
    CrsfLinkStats stats;
    TEST_ASSERT_TRUE(crsfDecodeLinkStats(payload, 10, stats));
    TEST_ASSERT_EQUAL_UINT8(100, stats.uplinkLq);
    TEST_ASSERT_EQUAL_INT8(-5, stats.uplinkSnr);
    TEST_ASSERT_FALSE(crsfDecodeLinkStats(payload, 9, stats));
}

static void test_telemetry_frames_parse_back()
{
    uint8_t frame[64];
    CrsfParser parser;

    uint8_t length = crsfBuildBatteryFrame(frame, sizeof(frame), 168, 125, 1300, 75);
    TEST_ASSERT_EQUAL_UINT8(12, length);
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame, length));
    TEST_ASSERT_EQUAL_HEX8(CRSF_FRAMETYPE_BATTERY, parser.frameType());
    TEST_ASSERT_EQUAL_HEX8(0x00, parser.payload()[0]);  // 168 big endian
    TEST_ASSERT_EQUAL_HEX8(0xA8, parser.payload()[1]);

    length = crsfBuildAttitudeFrame(frame, sizeof(frame), 0.1f, -0.2f, 1.0f);
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame, length));
    TEST_ASSERT_EQUAL_HEX8(CRSF_FRAMETYPE_ATTITUDE, parser.frameType());

    length = crsfBuildFlightModeFrame(frame, sizeof(frame), "ACRO");
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame, length));
    TEST_ASSERT_EQUAL_STRING("ACRO", (const char*)parser.payload());
}

static void test_telemetry_too_small_buffer()
{
    uint8_t frame[8];
    TEST_ASSERT_EQUAL_UINT8(0, crsfBuildBatteryFrame(frame, sizeof(frame), 1, 2, 3, 4));
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_parses_rc_frame_and_unpacks_channels);
    RUN_TEST(test_frame_split_across_feeds);
    RUN_TEST(test_rejects_bad_crc);
    RUN_TEST(test_resyncs_after_garbage);
    RUN_TEST(test_rejects_bad_length);
    RUN_TEST(test_channel_to_us);
    RUN_TEST(test_link_stats_decode);
    RUN_TEST(test_telemetry_frames_parse_back);
    RUN_TEST(test_telemetry_too_small_buffer);
    return UNITY_END();
}
