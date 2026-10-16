// Flight data recorder on an SD card. One file per arm (LOG00001.BBL, ...).
//
// The control loop only copies records into a RAM ring buffer. service() writes one
// 512-byte block per call, so SD latency spikes never reach the loop. The next file
// is created and pre-allocated while disarmed, so arming doesn't wait for the card.
#pragma once

#include <stdint.h>
#include "bb_record.h"

namespace blackbox {

// Mounts the card and prepares the first file. Missing card = blackbox off, still flyable.
void init();

void setRateDivider(uint8_t divider);

// Starts a log (writes the header). Call on arm.
void start(uint32_t nowMs);
// Requests the end of the log. Closing happens in service(), after the motors stopped.
void requestStop();

void logLoop(const BbLoopRecord& record);
void logEvent(uint32_t timeUs, uint8_t level, uint8_t code, int32_t a, int32_t b);
void logPerf(const BbPerfRecord& record);

// Writes at most one block, or closes/prepares files. Call every loop.
void service(uint32_t nowMs);

bool available();
bool logging();
const char* fileName();
uint32_t bytesWritten();
uint32_t droppedRecords();
uint32_t bufferUsed();
uint32_t bufferSize();

}  // namespace blackbox
