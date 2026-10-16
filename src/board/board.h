// The only hardware interface the rest of the firmware sees.
// Each MCU implements it in its own folder (board/<mcu>/board.cpp) and
// platformio.ini compiles just the one for the current env.
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

namespace board {

static const int MOTOR_COUNT = 4;

struct Info {
    const char* name;
    uint16_t loopHz;           // main control loop rate this MCU can hold
    uint8_t blackboxRateDiv;   // log every Nth loop
};

const Info& info();

// Starts the debug serial, buses and LED. Motors are not touched here.
void init(uint32_t debugBaud);

uint32_t micros();
uint32_t millis();

// OneShot125 outputs. motorsInit() sends the stop pulse from its first line on,
// so ESCs never see an undefined signal.
void motorsInit();
// command 0..1 per motor, mapped to 125..250 us.
void motorsWrite(const float command[MOTOR_COUNT]);
void motorsStop();

void rcSerialBegin(uint32_t baud);
Stream& rcSerial();
Stream& debugSerial();
TwoWire& imuWire();
SPIClass& sdSpi();
uint8_t sdCsPin();

// Voltage at the battery ADC pin, before the divider is undone.
float batteryPinVolts();

void ledWrite(bool on);

void watchdogInit(uint32_t timeoutMs);
void watchdogKick();
bool resetWasWatchdog();
const char* resetReason();
void reboot();

uint32_t freeHeap();

// After this call any heap allocation halts with fatal() instead of quietly
// fragmenting memory. Before it, vendor cores may allocate during startup.
void heapGuardArm();

// Static RAM reserved for the blackbox ring buffer, sized per MCU.
uint8_t* blackboxBuffer(uint32_t& size);

// Persistent storage for one parameter blob. Returns false if nothing valid could be read/written.
bool storageLoad(uint8_t* data, uint32_t size);
bool storageSave(const uint8_t* data, uint32_t size);

// Stops the motors, prints the reason and halts with an SOS blink. Never returns.
void fatal(const char* reason);

}  // namespace board
