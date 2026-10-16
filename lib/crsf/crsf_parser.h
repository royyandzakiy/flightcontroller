// Byte-by-byte CRSF frame parser. Feed it every received byte, it reports each frame
// that arrives complete with a valid CRC and silently resynchronizes on garbage.
#pragma once

#include <stdint.h>
#include "crsf_protocol.h"

struct CrsfLinkStats {
    uint8_t uplinkRssi1;     // dBm, sign dropped by the protocol (value 90 means -90 dBm)
    uint8_t uplinkRssi2;
    uint8_t uplinkLq;        // link quality, 0..100 %
    int8_t uplinkSnr;        // dB
    uint8_t activeAntenna;
    uint8_t rfMode;
    uint8_t uplinkTxPower;   // enum, not mW
    uint8_t downlinkRssi;
    uint8_t downlinkLq;
    int8_t downlinkSnr;
};

class CrsfParser {
public:
    CrsfParser();

    // Returns true when this byte completed a valid frame. The frame stays readable
    // through frameType() and payload() until the next byte is fed.
    bool feed(uint8_t byte);

    uint8_t frameType() const { return frame_[0]; }
    const uint8_t* payload() const { return &frame_[1]; }
    uint8_t payloadLength() const { return (uint8_t)(length_ - 2); }

    uint32_t goodFrames() const { return goodFrames_; }
    uint32_t badCrcFrames() const { return badCrcFrames_; }
    uint32_t badLengthFrames() const { return badLengthFrames_; }

private:
    enum class State : uint8_t { WaitAddress, WaitLength, ReadFrame };

    State state_;
    uint8_t length_;    // type + payload + crc
    uint8_t position_;  // bytes of frame_ filled so far
    uint8_t frame_[CRSF_LENGTH_MAX];
    uint32_t goodFrames_;
    uint32_t badCrcFrames_;
    uint32_t badLengthFrames_;
};

// Unpacks the 16 channels of an RC_CHANNELS_PACKED payload (11 bits each, LSB first).
void crsfUnpackChannels(const uint8_t* payload, uint16_t channels[CRSF_RC_CHANNEL_COUNT]);

// Converts a raw channel value (172..1811) to a servo pulse width (988..2012 us).
uint16_t crsfChannelToUs(uint16_t raw);

// Returns false if the payload is too short.
bool crsfDecodeLinkStats(const uint8_t* payload, uint8_t length, CrsfLinkStats& out);
