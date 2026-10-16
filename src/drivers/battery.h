// Battery voltage through a resistor divider on an ADC pin.
#pragma once

#include <stdint.h>

enum class BatteryState : uint8_t { Absent, Ok, Low, Critical };

namespace battery {

void configure(float dividerScale, float cellLowVolts, float cellCriticalVolts);

// Reads and filters the voltage, detects cell count, posts an event on state change.
// Call at a fixed rate (50 Hz).
void update();

float volts();
uint16_t millivolts();
uint8_t cells();
BatteryState state();
const char* stateName(BatteryState state);

}  // namespace battery
