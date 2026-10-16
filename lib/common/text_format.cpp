#include "text_format.h"

#include <stdio.h>
#include "fc_math.h"

uint32_t fcFormatFloat(char* out, uint32_t capacity, float value, uint8_t decimals)
{
    if (capacity == 0) return 0;
    if (!fcIsFinite(value)) {
        int written = snprintf(out, capacity, "nan");
        return (written < 0) ? 0 : (uint32_t)written;
    }
    if (decimals > 6) decimals = 6;

    uint32_t scale = 1;
    for (uint8_t i = 0; i < decimals; i++) scale *= 10;

    bool negative = value < 0.0f;
    float magnitude = negative ? -value : value;
    // Rounded in 64 bits so values up to ~1e12 still fit after scaling.
    uint64_t scaled = (uint64_t)(magnitude * (float)scale + 0.5f);
    unsigned long whole = (unsigned long)(scaled / scale);
    unsigned long fraction = (unsigned long)(scaled % scale);

    int written;
    if (decimals == 0) {
        written = snprintf(out, capacity, "%s%lu", negative ? "-" : "", whole);
    } else {
        written = snprintf(out, capacity, "%s%lu.%0*lu", negative ? "-" : "", whole, (int)decimals, fraction);
    }
    if (written < 0) {
        out[0] = '\0';
        return 0;
    }
    return ((uint32_t)written < capacity) ? (uint32_t)written : capacity - 1;
}

bool fcParseFloat(const char* text, float& value)
{
    const char* p = text;
    bool negative = false;
    if (*p == '-' || *p == '+') {
        negative = (*p == '-');
        p++;
    }

    float result = 0.0f;
    bool anyDigit = false;
    while (*p >= '0' && *p <= '9') {
        result = result * 10.0f + (float)(*p - '0');
        anyDigit = true;
        p++;
    }
    if (*p == '.') {
        p++;
        float place = 0.1f;
        while (*p >= '0' && *p <= '9') {
            result += (float)(*p - '0') * place;
            place *= 0.1f;
            anyDigit = true;
            p++;
        }
    }
    if (!anyDigit) return false;

    if (*p == 'e' || *p == 'E') {
        p++;
        bool exponentNegative = false;
        if (*p == '-' || *p == '+') {
            exponentNegative = (*p == '-');
            p++;
        }
        if (!(*p >= '0' && *p <= '9')) return false;
        int exponent = 0;
        while (*p >= '0' && *p <= '9') {
            if (exponent < 100) exponent = exponent * 10 + (*p - '0');
            p++;
        }
        for (int i = 0; i < exponent; i++) {
            result = exponentNegative ? result / 10.0f : result * 10.0f;
        }
    }
    if (*p != '\0') return false;

    value = negative ? -result : result;
    return fcIsFinite(value);
}
