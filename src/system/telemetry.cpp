#include "system/telemetry.h"

#include "crsf_protocol.h"
#include "crsf_telemetry.h"
#include "drivers/battery.h"
#include "drivers/rc_crsf.h"
#include "fc_math.h"

namespace {

int nextFrame = 0;

}  // namespace

namespace telemetry {

void update(const Angles& attitude, const char* flightModeName)
{
    // Telemetry only makes sense with a receiver that is listening.
    if (!rc::linkOk()) return;

    uint8_t frame[CRSF_FRAME_SIZE_MAX];
    uint8_t length = 0;
    switch (nextFrame) {
    case 0:
        length = crsfBuildBatteryFrame(frame, sizeof(frame), (uint16_t)(battery::millivolts() / 100),
                                       0, 0, 0);
        break;
    case 1:
        length = crsfBuildAttitudeFrame(frame, sizeof(frame), attitude.pitchDeg * FC_DEG_TO_RAD,
                                        attitude.rollDeg * FC_DEG_TO_RAD, 0.0f);
        break;
    default:
        length = crsfBuildFlightModeFrame(frame, sizeof(frame), flightModeName);
        break;
    }
    nextFrame = (nextFrame + 1) % 3;
    if (length > 0) rc::send(frame, length);
}

}  // namespace telemetry
