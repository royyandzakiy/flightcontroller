#include <Arduino.h>

// =====================================
// BOARD CONFIGURATION
// =====================================
#ifdef BOARD_ESP32
    #define BOARD_NAME "ESP32"
#elif defined(BOARD_STM32)
    #define BOARD_NAME "STM32"
#else
    #error "No board defined! Please define BOARD_ESP32 or BOARD_STM32"
#endif

// =====================================
// PIN DEFINITIONS
// =====================================
#ifndef LED_PIN
    #ifdef BOARD_ESP32
        #define LED_PIN 2
    #else
        #define LED_PIN PC13
    #endif
#endif

// =====================================
// MODULE CONFIGURATION
// =====================================
// Timing intervals
#define LED_TOGGLE_INTERVAL    1000
#define GYRO_INTERVAL          50
#define ELRS_INTERVAL          50

// Gyro registers
#define GYRO_I2C_ADDRESS       0x68
#define PWR_MGMT_1_REGISTER    0x6B
#define GYRO_DATA_REGISTER     0x43

// ELRS
#define SBUS_BUFFER_SIZE       25
#define ELRS_SERIAL_BAUD       420000

// =====================================
// MODULE INCLUDES
// =====================================
// GYRO
#include <Wire.h>

// ELRS
#include <crsf.h>

// =====================================
// MODULE FORWARD DECLARATIONS
// =====================================
namespace LED {
    void setup();
    void loop();
}

namespace GYRO {
    void setup();
    void loop();
    void printValues();
}

namespace ELRS {
    void setup();
    void loop();
}

namespace PWM {
    void setup();
    void setAllPositions(int ailerons, int elevator, int throttle, int rudder, int switchVal);
}

// =====================================
// MAIN SETUP & LOOP
// =====================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.printf("Setup Start! Board: %s\n", BOARD_NAME);

    LED::setup();
    GYRO::setup();
    ELRS::setup();
    PWM::setup();
}

void loop() {
    GYRO::loop();
    ELRS::loop();
    LED::loop();
}

// =====================================
// LED MODULE
// =====================================
namespace LED {
    long nextToggleTime = 0;
    bool ledState = false;

    void setup() {
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, LOW);
    }

    void toggle() {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }

    void loop() {
        if (millis() > nextToggleTime) {
            toggle();
            nextToggleTime = millis() + LED_TOGGLE_INTERVAL;
        }
    }
}

// =====================================
// GYRO MODULE
// =====================================
namespace GYRO {
    long nextReadTime = 0;
    int16_t gyroX, gyroY, gyroZ;

    void setup() {
        Wire.begin();
        Wire.setClock(400000);
        delay(250);

        // Activate gyro
        Wire.beginTransmission(GYRO_I2C_ADDRESS);
        Wire.write(PWR_MGMT_1_REGISTER);
        Wire.write(0x00);
        Wire.endTransmission();
    }

    void readValues() {
        Wire.beginTransmission(GYRO_I2C_ADDRESS);
        Wire.write(GYRO_DATA_REGISTER);
        Wire.endTransmission();
        
        Wire.requestFrom(GYRO_I2C_ADDRESS, 6);
        gyroX = Wire.read() << 8 | Wire.read();
        gyroY = Wire.read() << 8 | Wire.read();
        gyroZ = Wire.read() << 8 | Wire.read();
    }

    void printValues() {
        readValues();
        Serial.printf("GYRO - X: %d; Y: %d; Z: %d;\n", gyroX, gyroY, gyroZ);
    }

    void loop() {
        if (millis() > nextReadTime) {
            printValues();
            nextReadTime = millis() + GYRO_INTERVAL;
        }
    }
}

// =====================================
// ELRS MODULE
// =====================================
namespace ELRS {
    long nextReadTime = 0;
    uint8_t sbusBuffer[SBUS_BUFFER_SIZE] = {};
    uint16_t rcValues[RC_INPUT_MAX_CHANNELS] = {};
    uint16_t rcCount = 0;
    
    #ifdef BOARD_STM32
        HardwareSerial Serial3(PB11, PB10);
    #endif

    void setup() {
        #ifdef BOARD_STM32
            Serial3.begin(ELRS_SERIAL_BAUD, SERIAL_8N1);
        #else
            // ESP32 uses predefined Serial2 or similar
            // Adjust based on your ESP32 pin configuration
        #endif
    }

    void loop() {
        if (millis() > nextReadTime) {
            size_t bytesRead = 0;
            #ifdef BOARD_STM32
                if (Serial3.available()) {
                    bytesRead = Serial3.readBytes(sbusBuffer, SBUS_BUFFER_SIZE);
            #else
                // ESP32 serial reading implementation
                // This needs to be adapted based on your ESP32 setup
            #endif
            
            if (bytesRead > 0) {
                crsf_parse(sbusBuffer, SBUS_BUFFER_SIZE, rcValues, &rcCount, RC_INPUT_MAX_CHANNELS);

                // Map RC values (1000-2000 range) to PWM percentages (0-100)
                int ailerons = map(rcValues[0], 1000, 2000, 0, 100);
                int elevator = map(rcValues[1], 1000, 2000, 0, 100);
                int throttle = map(rcValues[2], 1000, 2000, 0, 100);
                int rudder = map(rcValues[3], 1000, 2000, 0, 100);
                int switchVal = map(rcValues[4], 1000, 2000, 0, 100);

                Serial.printf(
                    "CH1: %d (Ail: %d); CH2: %d (Ele: %d); CH3: %d (Thr: %d); CH4: %d (Rud: %d); CH5: %d (Swt: %d); ",
                    rcValues[0], ailerons,
                    rcValues[1], elevator,
                    rcValues[2], throttle,
                    rcValues[3], rudder,
                    rcValues[4], switchVal
                );

                PWM::setAllPositions(ailerons, elevator, throttle, rudder, switchVal);
            }

            nextReadTime = millis() + ELRS_INTERVAL;
        }
    }
}

// =====================================
// PWM MODULE
// =====================================
namespace PWM {
    // Pin definitions
    #ifdef BOARD_ESP32
        const int AILERONS_PIN = 12;
        const int ELEVATOR_PIN = 13;
        const int THROTTLE_PIN = 14;
        const int RUDDER_PIN = 15;
    #else
        // STM32 pin definitions
        const int AILERONS_PIN = 12;
        const int ELEVATOR_PIN = 13;
        const int THROTTLE_PIN = 14;
        const int RUDDER_PIN = 15;
    #endif

    // PWM channels
    const int AILERONS_CHANNEL = 1;
    const int ELEVATOR_CHANNEL = 2;
    const int THROTTLE_CHANNEL = 3;
    const int RUDDER_CHANNEL = 4;
    const int SWITCH_CHANNEL = 5;

    void setup() {
        #ifdef BOARD_ESP32
            // ESP32 PWM setup
            ledcSetup(AILERONS_CHANNEL, 50, 16);
            ledcSetup(ELEVATOR_CHANNEL, 50, 16);
            ledcSetup(THROTTLE_CHANNEL, 50, 16);
            ledcSetup(RUDDER_CHANNEL, 50, 16);

            ledcAttachPin(AILERONS_PIN, AILERONS_CHANNEL);
            ledcAttachPin(ELEVATOR_PIN, ELEVATOR_CHANNEL);
            ledcAttachPin(THROTTLE_PIN, THROTTLE_CHANNEL);
            ledcAttachPin(RUDDER_PIN, RUDDER_CHANNEL);
        #else
            // STM32 PWM setup - implement according to your STM32 library
            // pinMode(AILERONS_PIN, OUTPUT);
            // etc...
        #endif
    }

    void setPosition(float percent, int pwmChannel) {
        #ifdef BOARD_ESP32
            // ESP32 PWM implementation
            uint32_t duty = map(percent, 0, 100, 3276.8, 6553.6);
            ledcWrite(pwmChannel, duty);
        #else
            // STM32 PWM implementation
            // Add your STM32 PWM code here
        #endif
    }

    void setAllPositions(int ailerons, int elevator, int throttle, int rudder, int switchVal) {
        setPosition(ailerons, AILERONS_CHANNEL);
        setPosition(elevator, ELEVATOR_CHANNEL);
        setPosition(throttle, THROTTLE_CHANNEL);
        setPosition(rudder, RUDDER_CHANNEL);
        setPosition(switchVal, SWITCH_CHANNEL);
    }
}