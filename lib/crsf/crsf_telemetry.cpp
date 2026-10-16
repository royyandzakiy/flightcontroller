#include "crsf_telemetry.h"

#include "byte_io.h"
#include "crc.h"
#include "crsf_protocol.h"
#include "fc_math.h"

// Fills in the length byte and appends the CRC once the payload is written.
static uint8_t finishFrame(ByteWriter& writer)
{
    if (writer.overflowed()) return 0;
    uint8_t* frame = writer.data();
    // Length counts type + payload + crc, which is everything after the first two bytes.
    frame[1] = (uint8_t)(writer.length() - 2 + 1);
    writer.u8(crc8DvbS2Buffer(&frame[2], writer.length() - 2));
    if (writer.overflowed()) return 0;
    return (uint8_t)writer.length();
}

static void startFrame(ByteWriter& writer, uint8_t type)
{
    writer.u8(CRSF_ADDRESS_FLIGHT_CONTROLLER);
    writer.u8(0);  // length, set by finishFrame()
    writer.u8(type);
}

uint8_t crsfBuildBatteryFrame(uint8_t* out, uint8_t capacity,
                              uint16_t voltageDecivolts, uint16_t currentDeciamps,
                              uint32_t usedMah, uint8_t remainingPercent)
{
    ByteWriter writer(out, capacity);
    startFrame(writer, CRSF_FRAMETYPE_BATTERY);
    writer.u16be(voltageDecivolts);
    writer.u16be(currentDeciamps);
    writer.u24be(usedMah);
    writer.u8(remainingPercent);
    return finishFrame(writer);
}

uint8_t crsfBuildAttitudeFrame(uint8_t* out, uint8_t capacity,
                               float pitchRad, float rollRad, float yawRad)
{
    ByteWriter writer(out, capacity);
    startFrame(writer, CRSF_FRAMETYPE_ATTITUDE);
    writer.i16be((int16_t)fcClampInt(fcRound(pitchRad * 10000.0f), -32768, 32767));
    writer.i16be((int16_t)fcClampInt(fcRound(rollRad * 10000.0f), -32768, 32767));
    writer.i16be((int16_t)fcClampInt(fcRound(yawRad * 10000.0f), -32768, 32767));
    return finishFrame(writer);
}

uint8_t crsfBuildFlightModeFrame(uint8_t* out, uint8_t capacity, const char* mode)
{
    ByteWriter writer(out, capacity);
    startFrame(writer, CRSF_FRAMETYPE_FLIGHT_MODE);
    for (uint8_t i = 0; i < 15 && mode[i] != '\0'; i++) {
        writer.u8((uint8_t)mode[i]);
    }
    writer.u8(0);
    return finishFrame(writer);
}
