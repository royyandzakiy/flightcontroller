// Float-to-text without printf("%f"), which is large on the STM32 and not linked by default.
#pragma once

#include <stdint.h>

// Writes value with a fixed number of decimals (0..6), e.g. 1.5 with 3 -> "1.500".
// Returns the number of characters written, the output is always null-terminated.
uint32_t fcFormatFloat(char* out, uint32_t capacity, float value, uint8_t decimals);

// Parses "-12", "0.0025", "1e-3" and similar. Returns false if the whole text isn't a number.
// Used instead of strtof, which allocates heap memory in newlib.
bool fcParseFloat(const char* text, float& value);
