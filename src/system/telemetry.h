// CRSF telemetry to the radio: battery, attitude and flight mode, one frame per call.
#pragma once

#include "flight_types.h"

namespace telemetry {

void update(const Angles& attitude, const char* flightModeName);

}  // namespace telemetry
