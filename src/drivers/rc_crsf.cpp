#include "drivers/rc_crsf.h"

#include "board/board.h"
#include "config.h"
#include "crsf_protocol.h"
#include "fc_math.h"
#include "system/event_log.h"

namespace {

CrsfParser parser;
RcInput current;
uint16_t channelsUs[CRSF_RC_CHANNEL_COUNT];
CrsfLinkStats stats;
bool haveStats = false;
bool haveFrame = false;
bool linkWasOk = false;
bool isLinkOk = false;
uint32_t lastFrameMs = 0;

float stickFromUs(uint16_t us)
{
    float value = fcClamp(((float)us - 1500.0f) / 500.0f, -1.0f, 1.0f);
    float magnitude = value < 0.0f ? -value : value;
    if (magnitude < RC_STICK_DEADBAND) return 0.0f;
    // Rescale so the stick still reaches exactly 1 at full travel.
    float scaled = (magnitude - RC_STICK_DEADBAND) / (1.0f - RC_STICK_DEADBAND);
    return value < 0.0f ? -scaled : scaled;
}

SwitchPos switchFromUs(uint16_t us)
{
    if (us < 1300) return SwitchPos::Low;
    if (us > 1700) return SwitchPos::High;
    return SwitchPos::Mid;
}

void decodeChannels(const uint8_t* payload)
{
    uint16_t raw[CRSF_RC_CHANNEL_COUNT];
    crsfUnpackChannels(payload, raw);
    for (int i = 0; i < CRSF_RC_CHANNEL_COUNT; i++) channelsUs[i] = crsfChannelToUs(raw[i]);

    current.stick[AXIS_ROLL] = stickFromUs(channelsUs[RC_CHANNEL_ROLL]);
    // Pitch stick forward sends a high value but means nose down, which is negative pitch.
    current.stick[AXIS_PITCH] = -stickFromUs(channelsUs[RC_CHANNEL_PITCH]);
    current.stick[AXIS_YAW] = stickFromUs(channelsUs[RC_CHANNEL_YAW]);
    current.throttle = fcClamp(((float)channelsUs[RC_CHANNEL_THROTTLE] - 1000.0f) / 1000.0f, 0.0f, 1.0f);
    for (int i = 0; i < RC_AUX_COUNT; i++) {
        current.aux[i] = switchFromUs(channelsUs[RC_CHANNEL_AUX1 + i]);
    }
}

}  // namespace

namespace rc {

void init()
{
    for (int axis = 0; axis < AXIS_COUNT; axis++) current.stick[axis] = 0.0f;
    current.throttle = 0.0f;
    for (int i = 0; i < RC_AUX_COUNT; i++) current.aux[i] = SwitchPos::Low;
    for (int i = 0; i < CRSF_RC_CHANNEL_COUNT; i++) channelsUs[i] = 0;
    board::rcSerialBegin(CRSF_BAUD);
}

void update(uint32_t nowMs)
{
    Stream& serial = board::rcSerial();
    uint32_t budget = RC_MAX_BYTES_PER_UPDATE;
    while (budget > 0 && serial.available() > 0) {
        int byte = serial.read();
        budget--;
        if (byte < 0) break;
        if (!parser.feed((uint8_t)byte)) continue;

        if (parser.frameType() == CRSF_FRAMETYPE_RC_CHANNELS_PACKED
            && parser.payloadLength() == CRSF_RC_PAYLOAD_SIZE) {
            decodeChannels(parser.payload());
            haveFrame = true;
            lastFrameMs = nowMs;
        } else if (parser.frameType() == CRSF_FRAMETYPE_LINK_STATISTICS) {
            haveStats = crsfDecodeLinkStats(parser.payload(), parser.payloadLength(), stats);
        }
    }

    bool fresh = haveFrame && (nowMs - lastFrameMs) <= RC_LINK_TIMEOUT_MS;
    // Some receivers keep sending frames after losing the transmitter, LQ 0 says so.
    bool receiverSaysDead = haveStats && stats.uplinkLq == 0;
    isLinkOk = fresh && !receiverSaysDead;

    if (isLinkOk != linkWasOk) {
        eventLog::post(isLinkOk ? LogLevel::Info : LogLevel::Warn,
                       isLinkOk ? LogCode::RcLinkRestored : LogCode::RcLinkLost);
        linkWasOk = isLinkOk;
    }
}

const RcInput& input()
{
    return current;
}

bool linkOk()
{
    return isLinkOk;
}

uint16_t channelUs(int channel)
{
    if (channel < 0 || channel >= CRSF_RC_CHANNEL_COUNT) return 0;
    return channelsUs[channel];
}

const CrsfLinkStats& linkStats()
{
    return stats;
}

uint32_t goodFrames()
{
    return parser.goodFrames();
}

uint32_t badFrames()
{
    return parser.badCrcFrames() + parser.badLengthFrames();
}

bool send(const uint8_t* frame, uint8_t length)
{
    Stream& serial = board::rcSerial();
    if (serial.availableForWrite() < length) return false;
    serial.write(frame, length);
    return true;
}

}  // namespace rc
