#include "byte_ring_buffer.h"

ByteRingBuffer::ByteRingBuffer()
    : storage_(0), size_(0), head_(0), tail_(0), used_(0)
{
}

void ByteRingBuffer::init(uint8_t* storage, uint32_t size)
{
    storage_ = storage;
    size_ = size;
    clear();
}

void ByteRingBuffer::clear()
{
    head_ = 0;
    tail_ = 0;
    used_ = 0;
}

bool ByteRingBuffer::push(const uint8_t* data, uint32_t length)
{
    if (length > space()) {
        return false;
    }
    for (uint32_t i = 0; i < length; i++) {
        storage_[head_] = data[i];
        head_++;
        if (head_ == size_) head_ = 0;
    }
    used_ += length;
    return true;
}

uint32_t ByteRingBuffer::pop(uint8_t* out, uint32_t maxLength)
{
    uint32_t count = (maxLength < used_) ? maxLength : used_;
    for (uint32_t i = 0; i < count; i++) {
        out[i] = storage_[tail_];
        tail_++;
        if (tail_ == size_) tail_ = 0;
    }
    used_ -= count;
    return count;
}
