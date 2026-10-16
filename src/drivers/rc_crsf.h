// Radio input from an ExpressLRS/CRSF receiver.
#pragma once

#include <stdint.h>
#include "crsf_parser.h"
#include "flight_types.h"

namespace rc {

void init();

// Reads everything the receiver sent since the last call. Never blocks.
void update(uint32_t nowMs);

// Last valid input. Sticks hold their last value while the link is down.
const RcInput& input();

// A valid RC frame arrived recently and the receiver doesn't report a dead link.
bool linkOk();

uint16_t channelUs(int channel);
const CrsfLinkStats& linkStats();
uint32_t goodFrames();
uint32_t badFrames();

// Sends a complete CRSF frame back to the receiver (telemetry). Skipped if the UART is busy.
bool send(const uint8_t* frame, uint8_t length);

}  // namespace rc
