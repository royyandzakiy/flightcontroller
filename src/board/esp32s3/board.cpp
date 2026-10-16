// Board support for the ESP32-S3 DevKitC-1 (arduino-esp32 v3).
#include "board/board.h"
#include "board/board_common.h"

#include <Preferences.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

namespace {

const uint8_t MOTOR_PINS[board::MOTOR_COUNT] = { 4, 5, 6, 7 };
const uint8_t PIN_I2C_SDA = 8;
const uint8_t PIN_I2C_SCL = 9;
const uint8_t PIN_RC_RX = 18;
const uint8_t PIN_RC_TX = 17;
const uint8_t PIN_SD_SCK = 12;
const uint8_t PIN_SD_MISO = 13;
const uint8_t PIN_SD_MOSI = 11;
const uint8_t PIN_SD_CS = 10;
const uint8_t PIN_BATTERY_ADC = 1;
const uint8_t PIN_LED = 2;

// 2 kHz leaves room for the 250 us max pulse. 14 bits at 2 kHz gives ~0.03 us steps
// and stays under the 80 MHz LEDC clock limit (2000 * 2^14 = 32.8 MHz).
const uint32_t ONESHOT_FREQUENCY_HZ = 2000;
const uint8_t ONESHOT_RESOLUTION_BITS = 14;
const float ONESHOT_PERIOD_US = 1000000.0f / ONESHOT_FREQUENCY_HZ;

const uint32_t RC_RX_BUFFER_SIZE = 256;
const char* const STORAGE_NAMESPACE = "fc";
const char* const STORAGE_KEY = "params";

const board::Info INFO = { "ESP32-S3", 1000, 2 };

SPIClass sdSpiBus(FSPI);
Preferences preferences;
bool watchdogRunning = false;
bool heapGuardArmed = false;
uint8_t blackboxStorage[16384];

void writePulse(uint8_t pin, float pulseUs)
{
    uint32_t duty = (uint32_t)(pulseUs / ONESHOT_PERIOD_US * (float)(1u << ONESHOT_RESOLUTION_BITS));
    ledcWrite(pin, duty);
}

}  // namespace

namespace board {

const Info& info()
{
    return INFO;
}

void init(uint32_t debugBaud)
{
    Serial.begin(debugBaud);

    pinMode(PIN_LED, OUTPUT);
    ledWrite(false);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    // A stuck bus fails a read after 5 ms instead of the 50 ms default.
    Wire.setTimeOut(5);

    sdSpiBus.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

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
    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledcAttach(MOTOR_PINS[i], ONESHOT_FREQUENCY_HZ, ONESHOT_RESOLUTION_BITS);
    }
    motorsStop();
}

void motorsWrite(const float command[MOTOR_COUNT])
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        writePulse(MOTOR_PINS[i], oneshotPulseUs(command[i]));
    }
}

void motorsStop()
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        writePulse(MOTOR_PINS[i], ONESHOT_MIN_US);
    }
}

void rcSerialBegin(uint32_t baud)
{
    // Must be set before begin(), the driver allocates the buffer there.
    Serial1.setRxBufferSize(RC_RX_BUFFER_SIZE);
    Serial1.begin(baud, SERIAL_8N1, PIN_RC_RX, PIN_RC_TX);
}

Stream& rcSerial()
{
    return Serial1;
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
    return sdSpiBus;
}

uint8_t sdCsPin()
{
    return PIN_SD_CS;
}

float batteryPinVolts()
{
    // Uses the factory ADC calibration, more accurate than raw counts.
    return analogReadMilliVolts(PIN_BATTERY_ADC) / 1000.0f;
}

void ledWrite(bool on)
{
    digitalWrite(PIN_LED, on ? HIGH : LOW);
}

void watchdogInit(uint32_t timeoutMs)
{
    esp_task_wdt_config_t config;
    config.timeout_ms = timeoutMs;
    config.idle_core_mask = 0;  // only watch the loop task, not the idle tasks
    config.trigger_panic = true;
    // The core may have started the task watchdog already, then only reconfigure it.
    if (esp_task_wdt_init(&config) == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_reconfigure(&config);
    }
    esp_task_wdt_add(NULL);
    watchdogRunning = true;
}

void watchdogKick()
{
    if (watchdogRunning) esp_task_wdt_reset();
}

bool resetWasWatchdog()
{
    esp_reset_reason_t reason = esp_reset_reason();
    return reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT || reason == ESP_RST_WDT;
}

const char* resetReason()
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    default: return "other";
    }
}

void reboot()
{
    ESP.restart();
}

uint32_t freeHeap()
{
    return ESP.getFreeHeap();
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
    if (!preferences.begin(STORAGE_NAMESPACE, true)) return false;
    size_t read = preferences.getBytes(STORAGE_KEY, data, size);
    preferences.end();
    return read == size;
}

bool storageSave(const uint8_t* data, uint32_t size)
{
    if (!preferences.begin(STORAGE_NAMESPACE, false)) return false;
    size_t written = preferences.putBytes(STORAGE_KEY, data, size);
    preferences.end();
    return written == size;
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

// Heap guard: the ESP32 core leaves the global operator new replaceable, so every
// C++ allocation passes through here. The IDF's own C allocations are not affected.
static void* guardedAllocate(size_t size)
{
    if (heapGuardArmed) board::fatal("heap allocation after setup");
    void* memory = malloc(size == 0 ? 1 : size);
    if (memory == NULL) board::fatal("out of memory");
    return memory;
}

void* operator new(size_t size)
{
    return guardedAllocate(size);
}

void* operator new[](size_t size)
{
    return guardedAllocate(size);
}

void operator delete(void* memory) noexcept
{
    free(memory);
}

void operator delete[](void* memory) noexcept
{
    free(memory);
}

void operator delete(void* memory, size_t) noexcept
{
    free(memory);
}

void operator delete[](void* memory, size_t) noexcept
{
    free(memory);
}
