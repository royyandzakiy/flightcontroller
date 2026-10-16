// MPU6050 IMU over I2C: gyro and accelerometer in body-frame physical units.
#pragma once

#include <stdint.h>
#include "flight_types.h"

namespace imu {

// Checks WHO_AM_I and configures the chip. False if no known chip answers.
bool init();

// Gyro and accel filter settings, from the parameters.
void configure(float gyroLpfHz, float loopHz, const float accelOffsetG[AXIS_COUNT]);

// Averages ~1 s of samples to find the gyro bias. Blocking, only call from setup().
// Returns false if the quad moved, then call it again.
bool calibrateGyro();

// Measures the accel offset with the quad sitting level. Blocking, only while disarmed.
bool calibrateAccel(float offsetOutG[AXIS_COUNT]);

// One burst read. On a failed read the last good sample is returned.
const ImuSample& read();

bool healthy();
bool gyroCalibrated();
uint32_t errorCount();
uint8_t whoAmI();

}  // namespace imu
