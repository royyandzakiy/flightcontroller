// Board support for the STM32F103 Blue Pill (STM32duino core).
#include "board/board.h"
#include "board/board_common.h"

// Only the buffered flash API, EEPROM.h would add an unused global object.
#include <utility/stm32_eeprom.h>
#include <IWatchdog.h>

// Heap guard: platformio.ini links with --wrap=_sbrk, so every heap growth (malloc and
// new alike) calls __wrap__sbrk first. The core's own operator new can't be replaced.
extern "C" void* __real__sbrk(int increment);
extern "C" void* __wrap__sbrk(int increment);

namespace {

// PA0..PA3 are TIM2 channels 1..4, so all motors share one timer.
const uint32_t MOTOR_PINS[board::MOTOR_COUNT] = { PA0, PA1, PA2, PA3 };
const uint32_t PIN_I2C_SDA = PB7;
const uint32_t PIN_I2C_SCL = PB6;
const uint32_t PIN_RC_RX = PB11;
const uint32_t PIN_RC_TX = PB10;
const uint32_t PIN_SD_CS = PA4;
const uint32_t PIN_BATTERY_ADC = PB0;
const uint32_t PIN_LED = PC13;

const uint32_t ONESHOT_PERIOD_US = 500;  // 2 kHz, room for the 250 us max pulse

const board::Info INFO = { "STM32F103", 500, 2 };

HardwareSerial rcUart(PIN_RC_RX, PIN_RC_TX);
HardwareTimer motorTimer(TIM2);
uint32_t timerTicksPerUs = 72;
bool watchdogRunning = false;
bool watchdogCausedReset = false;
bool heapGuardArmed = false;
// 20 KB RAM in total, 4 KB is what the blackbox can get.
uint8_t blackboxStorage[4096];

void writePulse(int motor, float pulseUs)
{
    uint32_t ticks = (uint32_t)(pulseUs * (float)timerTicksPerUs + 0.5f);
    motorTimer.setCaptureCompare(motor + 1, ticks, TICK_COMPARE_FORMAT);
}

}  // namespace

namespace board {

const Info& info()
{
    return INFO;
}

void init(uint32_t debugBaud)
{
    // Read before the watchdog starts, isReset(true) also clears the flag.
    watchdogCausedReset = IWatchdog.isReset(true);

    Serial.begin(debugBaud);

    pinMode(PIN_LED, OUTPUT);
    ledWrite(false);

    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);

    // SPI1 default pins: PA5 SCK, PA6 MISO, PA7 MOSI.
    SPI.begin();

    analogReadResolution(12);
}

uint32_t micros()
{
    return (uint32_t)::micros();
}

uint32_t millis()
{
    return (uint32_t)::millis();
}

void motorsInit()
{
    motorTimer.setPrescaleFactor(1);
    timerTicksPerUs = motorTimer.getTimerClkFreq() / 1000000;
    motorTimer.setOverflow(ONESHOT_PERIOD_US * timerTicksPerUs, TICK_FORMAT);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motorTimer.setMode(i + 1, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_PINS[i]);
    }
    motorsStop();
    motorTimer.resume();
}

void motorsWrite(const float command[MOTOR_COUNT])
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        writePulse(i, oneshotPulseUs(command[i]));
    }
}

void motorsStop()
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        writePulse(i, ONESHOT_MIN_US);
    }
}

void rcSerialBegin(uint32_t baud)
{
    rcUart.begin(baud, SERIAL_8N1);
}

Stream& rcSerial()
{
    return rcUart;
}

Stream& debugSerial()
{
    return Serial;
}

TwoWire& imuWire()
{
    return Wire;
}

SPIClass& sdSpi()
{
    return SPI;
}

uint8_t sdCsPin()
{
    return (uint8_t)PIN_SD_CS;
}

float batteryPinVolts()
{
    return analogRead(PIN_BATTERY_ADC) * (3.3f / 4095.0f);
}

void ledWrite(bool on)
{
    // The Blue Pill LED is wired to 3.3 V, so LOW turns it on.
    digitalWrite(PIN_LED, on ? LOW : HIGH);
}

void watchdogInit(uint32_t timeoutMs)
{
    IWatchdog.begin(timeoutMs * 1000);
    watchdogRunning = true;
}

void watchdogKick()
{
    if (watchdogRunning) IWatchdog.reload();
}

bool resetWasWatchdog()
{
    return watchdogCausedReset;
}

const char* resetReason()
{
    return watchdogCausedReset ? "watchdog" : "power-on/other";
}

void reboot()
{
    NVIC_SystemReset();
}

uint32_t freeHeap()
{
    // Gap between the top of the heap and the stack: what is left for both to grow.
    char stackMarker;
    char* heapEnd = static_cast<char*>(__real__sbrk(0));
    return (uint32_t)(&stackMarker - heapEnd);
}

void heapGuardArm()
{
    heapGuardArmed = true;
}

uint8_t* blackboxBuffer(uint32_t& size)
{
    size = sizeof(blackboxStorage);
    return blackboxStorage;
}

bool storageLoad(uint8_t* data, uint32_t size)
{
    if (size > (uint32_t)E2END + 1) return false;
    eeprom_buffer_fill();
    for (uint32_t i = 0; i < size; i++) {
        data[i] = eeprom_buffered_read_byte(i);
    }
    return true;
}

bool storageSave(const uint8_t* data, uint32_t size)
{
    if (size > (uint32_t)E2END + 1) return false;
    // Buffered API: one flash page erase for the whole blob instead of one per byte.
    eeprom_buffer_fill();
    for (uint32_t i = 0; i < size; i++) {
        eeprom_buffered_write_byte(i, data[i]);
    }
    eeprom_buffer_flush();
    return true;
}

void fatal(const char* reason)
{
    motorsStop();
    uint32_t step = 0;
    uint32_t lastPrintMs = 0;
    while (true) {
        uint32_t now = millis();
        if (step == 0 || now - lastPrintMs >= 1000) {
            Serial.print("FATAL: ");
            Serial.println(reason);
            lastPrintMs = now;
        }
        ledWrite(SOS_PATTERN[step % sizeof(SOS_PATTERN)] != 0);
        step++;
        // Kept alive on purpose: a reset could reboot into a loop mid-air,
        // here the motors stay stopped and the reason stays readable.
        watchdogKick();
        delay(SOS_STEP_MS);
    }
}

}  // namespace board

extern "C" void* __wrap__sbrk(int increment)
{
    if (heapGuardArmed && increment > 0) board::fatal("heap allocation after setup");
    return __real__sbrk(increment);
}
