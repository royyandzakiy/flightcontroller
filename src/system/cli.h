// Serial command line on the debug port. Only accepted while disarmed, and an open
// session blocks arming. Type "help" for the command list.
#pragma once

#include <stdint.h>
#include "attitude.h"
#include "mode_manager.h"

namespace cli {

void init(const ModeManager& modes, const AttitudeEstimator& attitude);

// Reads input and runs complete command lines. Call from a slow task.
void update(uint32_t nowMs, bool armed);

bool active();

// Motor test started with "motor <n> <percent> props-off", stops by itself after 2 s.
bool motorTestActive(uint32_t nowMs);
int motorTestIndex();
float motorTestValue();
void stopMotorTest();

}  // namespace cli
