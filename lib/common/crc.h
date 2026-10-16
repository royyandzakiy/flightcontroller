// Checksums used by the radio protocol and the parameter storage.
#pragma once

#include <stdint.h>

// CRC-8 DVB-S2 (polynomial 0xD5), the checksum CRSF frames carry.
uint8_t crc8DvbS2(uint8_t crc, uint8_t value);
uint8_t crc8DvbS2Buffer(const uint8_t* data, uint32_t length);

// CRC-16 CCITT (polynomial 0x1021, start 0xFFFF).
uint16_t crc16Ccitt(const uint8_t* data, uint32_t length);
