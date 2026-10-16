// Status LED patterns:
//   armed            solid on
//   failsafe         fast blink (10 Hz)
//   ready to arm     short flash once per second
//   arm blocked      N blinks then a pause, N = lowest blocking flag bit + 1
#pragma once

#include <stdint.h>
#include "mode_manager.h"

namespace statusLed {

void update(uint32_t nowMs, ArmState state, uint16_t armBlockFlags);

}  // namespace statusLed
