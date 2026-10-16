// CRSF protocol constants (TBS Crossfire, used by ExpressLRS receivers).
// Frame layout: [address][length][type][payload...][crc], length counts type + payload + crc.
#pragma once

#include <stdint.h>

static const uint32_t CRSF_BAUD = 420000;

// The flight controller address doubles as the sync byte receivers start frames with.
static const uint8_t CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8;
static const uint8_t CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA;
static const uint8_t CRSF_ADDRESS_RECEIVER = 0xEC;
static const uint8_t CRSF_ADDRESS_TRANSMITTER = 0xEE;

static const uint8_t CRSF_LENGTH_MIN = 2;   // type + crc, no payload
static const uint8_t CRSF_LENGTH_MAX = 62;  // whole frame is at most 64 bytes
static const uint8_t CRSF_FRAME_SIZE_MAX = 64;

static const uint8_t CRSF_FRAMETYPE_BATTERY = 0x08;
static const uint8_t CRSF_FRAMETYPE_LINK_STATISTICS = 0x14;
static const uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
static const uint8_t CRSF_FRAMETYPE_ATTITUDE = 0x1E;
static const uint8_t CRSF_FRAMETYPE_FLIGHT_MODE = 0x21;

static const uint8_t CRSF_RC_CHANNEL_COUNT = 16;
static const uint8_t CRSF_RC_PAYLOAD_SIZE = 22;  // 16 channels x 11 bits
static const uint8_t CRSF_LINK_STATS_PAYLOAD_SIZE = 10;

// Raw channel range sent by the receiver, maps to 988..2012 us.
static const uint16_t CRSF_CHANNEL_RAW_MIN = 172;
static const uint16_t CRSF_CHANNEL_RAW_MID = 992;
static const uint16_t CRSF_CHANNEL_RAW_MAX = 1811;
