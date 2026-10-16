// Fixed-size FIFO of bytes. The caller owns the storage array, so the RAM cost is
// visible where the buffer is declared. Not interrupt-safe: use from one context only.
#pragma once

#include <stdint.h>

class ByteRingBuffer {
public:
    ByteRingBuffer();

    void init(uint8_t* storage, uint32_t size);
    void clear();

    // Adds all bytes or none, so a record is never split by a full buffer.
    bool push(const uint8_t* data, uint32_t length);

    // Removes up to maxLength bytes, returns how many were copied.
    uint32_t pop(uint8_t* out, uint32_t maxLength);

    uint32_t used() const { return used_; }
    uint32_t space() const { return size_ - used_; }
    uint32_t size() const { return size_; }

private:
    uint8_t* storage_;
    uint32_t size_;
    uint32_t head_;  // next write position
    uint32_t tail_;  // next read position
    uint32_t used_;
};
