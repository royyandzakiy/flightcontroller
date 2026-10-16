#include "crsf_parser.h"

#include "crc.h"

static bool isKnownAddress(uint8_t byte)
{
    return byte == CRSF_ADDRESS_FLIGHT_CONTROLLER
        || byte == CRSF_ADDRESS_RADIO_TRANSMITTER
        || byte == CRSF_ADDRESS_RECEIVER
        || byte == CRSF_ADDRESS_TRANSMITTER;
}

CrsfParser::CrsfParser()
    : state_(State::WaitAddress), length_(0), position_(0),
      goodFrames_(0), badCrcFrames_(0), badLengthFrames_(0)
{
    frame_[0] = 0;
}

bool CrsfParser::feed(uint8_t byte)
{
    switch (state_) {
    case State::WaitAddress:
        if (isKnownAddress(byte)) {
            state_ = State::WaitLength;
        }
        return false;

    case State::WaitLength:
        if (byte < CRSF_LENGTH_MIN || byte > CRSF_LENGTH_MAX) {
            badLengthFrames_++;
            // The rejected byte may itself be the start of the next frame.
            state_ = isKnownAddress(byte) ? State::WaitLength : State::WaitAddress;
            return false;
        }
        length_ = byte;
        position_ = 0;
        state_ = State::ReadFrame;
        return false;

    case State::ReadFrame:
        frame_[position_++] = byte;
        if (position_ < length_) {
            return false;
        }
        state_ = State::WaitAddress;
        // CRC covers type and payload, the last byte is the CRC itself.
        if (crc8DvbS2Buffer(frame_, (uint32_t)(length_ - 1)) != frame_[length_ - 1]) {
            badCrcFrames_++;
            return false;
        }
        goodFrames_++;
        return true;
    }
    return false;
}

void crsfUnpackChannels(const uint8_t* payload, uint16_t channels[CRSF_RC_CHANNEL_COUNT])
{
    uint32_t bits = 0;
    uint8_t bitCount = 0;
    uint8_t channel = 0;

    for (uint8_t i = 0; i < CRSF_RC_PAYLOAD_SIZE && channel < CRSF_RC_CHANNEL_COUNT; i++) {
        bits |= (uint32_t)payload[i] << bitCount;
        bitCount += 8;
        while (bitCount >= 11 && channel < CRSF_RC_CHANNEL_COUNT) {
            channels[channel++] = (uint16_t)(bits & 0x7FF);
            bits >>= 11;
            bitCount -= 11;
        }
    }
}

uint16_t crsfChannelToUs(uint16_t raw)
{
    // 5/8 is the CRSF scale: 1639 raw steps cover 1024 us around a 992 -> 1500 us center.
    int32_t us = ((int32_t)raw - CRSF_CHANNEL_RAW_MID) * 5 / 8 + 1500;
    return (uint16_t)us;
}

bool crsfDecodeLinkStats(const uint8_t* payload, uint8_t length, CrsfLinkStats& out)
{
    if (length < CRSF_LINK_STATS_PAYLOAD_SIZE) {
        return false;
    }
    out.uplinkRssi1 = payload[0];
    out.uplinkRssi2 = payload[1];
    out.uplinkLq = payload[2];
    out.uplinkSnr = (int8_t)payload[3];
    out.activeAntenna = payload[4];
    out.rfMode = payload[5];
    out.uplinkTxPower = payload[6];
    out.downlinkRssi = payload[7];
    out.downlinkLq = payload[8];
    out.downlinkSnr = (int8_t)payload[9];
    return true;
}
