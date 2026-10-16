#include "byte_io.h"

ByteWriter::ByteWriter(uint8_t* buffer, uint32_t capacity)
    : buffer_(buffer), capacity_(capacity), length_(0), overflowed_(false)
{
}

bool ByteWriter::reserve(uint32_t count)
{
    if (overflowed_ || length_ + count > capacity_) {
        overflowed_ = true;
        return false;
    }
    return true;
}

void ByteWriter::u8(uint8_t value)
{
    if (!reserve(1)) return;
    buffer_[length_++] = value;
}

void ByteWriter::u16le(uint16_t value)
{
    if (!reserve(2)) return;
    buffer_[length_++] = (uint8_t)(value & 0xFF);
    buffer_[length_++] = (uint8_t)(value >> 8);
}

void ByteWriter::i16le(int16_t value)
{
    u16le((uint16_t)value);
}

void ByteWriter::u32le(uint32_t value)
{
    if (!reserve(4)) return;
    buffer_[length_++] = (uint8_t)(value & 0xFF);
    buffer_[length_++] = (uint8_t)((value >> 8) & 0xFF);
    buffer_[length_++] = (uint8_t)((value >> 16) & 0xFF);
    buffer_[length_++] = (uint8_t)(value >> 24);
}

void ByteWriter::i32le(int32_t value)
{
    u32le((uint32_t)value);
}

void ByteWriter::u16be(uint16_t value)
{
    if (!reserve(2)) return;
    buffer_[length_++] = (uint8_t)(value >> 8);
    buffer_[length_++] = (uint8_t)(value & 0xFF);
}

void ByteWriter::i16be(int16_t value)
{
    u16be((uint16_t)value);
}

void ByteWriter::u24be(uint32_t value)
{
    if (!reserve(3)) return;
    buffer_[length_++] = (uint8_t)((value >> 16) & 0xFF);
    buffer_[length_++] = (uint8_t)((value >> 8) & 0xFF);
    buffer_[length_++] = (uint8_t)(value & 0xFF);
}

void ByteWriter::bytes(const uint8_t* data, uint32_t length)
{
    if (!reserve(length)) return;
    for (uint32_t i = 0; i < length; i++) {
        buffer_[length_++] = data[i];
    }
}

ByteReader::ByteReader(const uint8_t* buffer, uint32_t length)
    : buffer_(buffer), length_(length), position_(0), overflowed_(false)
{
}

bool ByteReader::take(uint32_t count)
{
    if (overflowed_ || position_ + count > length_) {
        overflowed_ = true;
        return false;
    }
    return true;
}

uint8_t ByteReader::u8()
{
    if (!take(1)) return 0;
    return buffer_[position_++];
}

uint16_t ByteReader::u16le()
{
    if (!take(2)) return 0;
    uint16_t value = (uint16_t)(buffer_[position_] | (buffer_[position_ + 1] << 8));
    position_ += 2;
    return value;
}

int16_t ByteReader::i16le()
{
    return (int16_t)u16le();
}

uint32_t ByteReader::u32le()
{
    if (!take(4)) return 0;
    uint32_t value = (uint32_t)buffer_[position_]
                   | ((uint32_t)buffer_[position_ + 1] << 8)
                   | ((uint32_t)buffer_[position_ + 2] << 16)
                   | ((uint32_t)buffer_[position_ + 3] << 24);
    position_ += 4;
    return value;
}

int32_t ByteReader::i32le()
{
    return (int32_t)u32le();
}
