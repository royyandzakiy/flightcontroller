#include "drivers/status_led.h"

#include "board/board.h"

namespace {

int lowestSetBit(uint16_t flags)
{
    for (int bit = 0; bit < 16; bit++) {
        if (flags & (1u << bit)) return bit;
    }
    return -1;
}

bool blinkCodeOn(uint32_t nowMs, int blinks)
{
    const uint32_t blinkMs = 200;
    const uint32_t pauseMs = 1000;
    uint32_t cycleMs = (uint32_t)blinks * 2 * blinkMs + pauseMs;
    uint32_t t = nowMs % cycleMs;
    if (t >= (uint32_t)blinks * 2 * blinkMs) return false;
    return (t / blinkMs) % 2 == 0;
}

}  // namespace

namespace statusLed {

void update(uint32_t nowMs, ArmState state, uint16_t armBlockFlags)
{
    bool on;
    if (state == ArmState::Armed) {
        on = true;
    } else if (state == ArmState::Failsafe) {
        on = (nowMs / 50) % 2 == 0;
    } else if (armBlockFlags == 0) {
        on = (nowMs % 1000) < 100;
    } else {
        on = blinkCodeOn(nowMs, lowestSetBit(armBlockFlags) + 1);
    }
    board::ledWrite(on);
}

}  // namespace statusLed
