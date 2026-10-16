// Compile-time settings. Values worth tuning in the field are parameters
// (system/params.cpp), pins and loop rates belong to the board files.
#pragma once

#include <stdint.h>

static const uint32_t DEBUG_BAUD = 460800;

// Long enough for an SD card file close, short enough to catch a hung loop.
static const uint32_t WATCHDOG_TIMEOUT_MS = 250;

// No valid RC frame for this long = link lost (arming blocked, failsafe when armed).
static const uint32_t RC_LINK_TIMEOUT_MS = 250;
// Most bytes read from the receiver UART per loop, bounds the time rc::update() can take.
static const uint32_t RC_MAX_BYTES_PER_UPDATE = 128;

// CRSF channel index (0-based) for each function. AETR, the ExpressLRS default.
static const uint8_t RC_CHANNEL_ROLL = 0;
static const uint8_t RC_CHANNEL_PITCH = 1;
static const uint8_t RC_CHANNEL_THROTTLE = 2;
static const uint8_t RC_CHANNEL_YAW = 3;
static const uint8_t RC_CHANNEL_AUX1 = 4;  // aux 1..4 follow on channels 5..8

// Stick travel around center treated as exactly centered (fraction of half travel).
static const float RC_STICK_DEADBAND = 0.02f;

// IMU mounting: chip arrow rotated clockwise from forward (0, 90, 180 or 270 deg, seen
// from above), and whether the chip is mounted upside down.
static const int IMU_ALIGN_YAW_DEG = 0;
static const bool IMU_UPSIDE_DOWN = false;
// This many failed reads in a row mark the IMU as failed until reboot.
static const uint32_t IMU_MAX_CONSECUTIVE_ERRORS = 10;

static const uint32_t GYRO_CAL_SAMPLES = 1000;
// Max - min per axis during calibration. More means the quad moved and it retries.
static const float GYRO_CAL_MAX_SPREAD_DPS = 6.0f;

// Accelerometer correction speed of the attitude estimate.
static const float ATTITUDE_ACCEL_TIME_CONSTANT_S = 2.0f;

// Below this throttle the I-term is frozen, so it can't wind up while sitting on the ground.
static const float PID_I_THROTTLE_MIN = 0.15f;

// Arming is refused if the last second had more overruns than this.
static const uint32_t LOOP_OVERLOAD_OVERRUNS_PER_S = 5;

static const uint32_t ARM_BOOT_GRACE_MS = 2000;
static const uint32_t CLI_SESSION_TIMEOUT_MS = 60000;
static const uint32_t MOTOR_TEST_TIMEOUT_MS = 2000;

// Reserved on the SD card when a log file is created. 32 MB is ~15 min at 500 records/s.
static const uint32_t BLACKBOX_PREALLOCATE_BYTES = 32UL * 1024UL * 1024UL;
static const uint32_t BLACKBOX_SYNC_INTERVAL_MS = 1000;
