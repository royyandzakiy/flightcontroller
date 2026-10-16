// Builds CRSF telemetry frames for the radio. Each function writes a complete frame
// (sync, length, type, payload, CRC) and returns its length, or 0 if it didn't fit.
#pragma once

#include <stdint.h>

uint8_t crsfBuildBatteryFrame(uint8_t* out, uint8_t capacity,
                              uint16_t voltageDecivolts, uint16_t currentDeciamps,
                              uint32_t usedMah, uint8_t remainingPercent);

// Angles in radians, sent as rad * 10000.
uint8_t crsfBuildAttitudeFrame(uint8_t* out, uint8_t capacity,
                               float pitchRad, float rollRad, float yawRad);

// Shown on the radio screen, at most 15 characters.
uint8_t crsfBuildFlightModeFrame(uint8_t* out, uint8_t capacity, const char* mode);
