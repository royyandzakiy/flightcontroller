// Reads and writes integers in a byte buffer with an explicit byte order, so encoded
// data never depends on the CPU or on struct padding.
#pragma once

#include <stdint.h>

class ByteWriter {
public:
    ByteWriter(uint8_t* buffer, uint32_t capacity);

    void u8(uint8_t value);
    void u16le(uint16_t value);
    void i16le(int16_t value);
    void u32le(uint32_t value);
    void i32le(int32_t value);
    void u16be(uint16_t value);
    void i16be(int16_t value);
    void u24be(uint32_t value);
    void bytes(const uint8_t* data, uint32_t length);

    uint32_t length() const { return length_; }
    uint8_t* data() const { return buffer_; }
    // True if a write didn't fit. Further writes are ignored.
    bool overflowed() const { return overflowed_; }

private:
    bool reserve(uint32_t count);

    uint8_t* buffer_;
    uint32_t capacity_;
    uint32_t length_;
    bool overflowed_;
};

class ByteReader {
public:
    ByteReader(const uint8_t* buffer, uint32_t length);

    uint8_t u8();
    uint16_t u16le();
    int16_t i16le();
    uint32_t u32le();
    int32_t i32le();

    uint32_t position() const { return position_; }
    // True if a read went past the end. Reads past the end return 0.
    bool overflowed() const { return overflowed_; }

private:
    bool take(uint32_t count);

    const uint8_t* buffer_;
    uint32_t length_;
    uint32_t position_;
    bool overflowed_;
};
