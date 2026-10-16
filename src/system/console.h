// Text output to the debug serial that never blocks the control loop.
// Text goes into a buffer, flush() sends only what the UART can take right now.
#pragma once

#include <stdint.h>

namespace console {

void init();

void print(const char* text);
void printf(const char* format, ...) __attribute__((format(printf, 1, 2)));

// Sends as much buffered text as fits in the UART without waiting. Call every loop.
void flush();

// Waits until everything is sent. Only for setup() and disarmed CLI commands.
void flushBlocking();

// While blocking, a full buffer is drained instead of dropping text. Used by the CLI
// so long command output arrives complete.
void setBlocking(bool blocking);

uint32_t droppedBytes();

}  // namespace console
