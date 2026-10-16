#include "system/console.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board/board.h"
#include "byte_ring_buffer.h"

namespace {

uint8_t storage[1024];
ByteRingBuffer buffer;
bool blockingMode = false;
uint32_t dropped = 0;

void writeText(const char* text, uint32_t length)
{
    if (blockingMode) {
        // Drain in chunks so output longer than the buffer still gets through.
        while (length > 0) {
            uint32_t chunk = length;
            if (chunk > buffer.space()) chunk = buffer.space();
            if (chunk > 0) {
                buffer.push(reinterpret_cast<const uint8_t*>(text), chunk);
                text += chunk;
                length -= chunk;
            }
            if (length > 0) console::flushBlocking();
        }
        return;
    }
    if (!buffer.push(reinterpret_cast<const uint8_t*>(text), length)) {
        dropped += length;
    }
}

}  // namespace

namespace console {

void init()
{
    buffer.init(storage, sizeof(storage));
}

void print(const char* text)
{
    writeText(text, (uint32_t)strlen(text));
}

void printf(const char* format, ...)
{
    char line[128];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length < 0) return;
    if ((uint32_t)length >= sizeof(line)) length = sizeof(line) - 1;
    writeText(line, (uint32_t)length);
}

void flush()
{
    Stream& serial = board::debugSerial();
    int room = serial.availableForWrite();
    if (room <= 0) return;

    uint8_t chunk[64];
    uint32_t want = (uint32_t)room;
    if (want > sizeof(chunk)) want = sizeof(chunk);
    uint32_t count = buffer.pop(chunk, want);
    if (count > 0) serial.write(chunk, count);
}

void flushBlocking()
{
    Stream& serial = board::debugSerial();
    uint8_t chunk[64];
    while (buffer.used() > 0) {
        uint32_t count = buffer.pop(chunk, sizeof(chunk));
        serial.write(chunk, count);
        board::watchdogKick();
    }
    serial.flush();
}

void setBlocking(bool blocking)
{
    blockingMode = blocking;
}

uint32_t droppedBytes()
{
    return dropped;
}

}  // namespace console
