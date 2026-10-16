#include "drivers/imu_mpu6050.h"

#include "board/board.h"
#include "config.h"
#include "pt1_filter.h"
#include "system/event_log.h"

namespace {

const uint8_t I2C_ADDRESS = 0x68;

const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_CONFIG = 0x1A;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_WHO_AM_I = 0x75;

// MPU6050 answers 0x68. The rest are register-compatible clones and successors
// (MPU6500, MPU9250, ...) often sold on "MPU6050" breakout boards.
const uint8_t KNOWN_IDS[] = { 0x68, 0x70, 0x71, 0x72, 0x73, 0x98 };

const float GYRO_LSB_PER_DPS = 16.4f;   // +-2000 deg/s range
const float ACCEL_LSB_PER_G = 4096.0f;  // +-8 g range

ImuSample sample;
Pt1Filter gyroFilters[AXIS_COUNT];
float gyroBiasDps[AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
float accelOffsetG[AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
bool calibrated = false;
bool isHealthy = false;
uint8_t chipId = 0;
uint32_t totalErrors = 0;
uint32_t consecutiveErrors = 0;

bool writeRegister(uint8_t reg, uint8_t value)
{
    TwoWire& wire = board::imuWire();
    wire.beginTransmission(I2C_ADDRESS);
    wire.write(reg);
    wire.write(value);
    return wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t* out, uint8_t count)
{
    TwoWire& wire = board::imuWire();
    wire.beginTransmission(I2C_ADDRESS);
    wire.write(reg);
    // false = repeated start, the bus stays claimed between address and data.
    if (wire.endTransmission(false) != 0) return false;
    if (wire.requestFrom(I2C_ADDRESS, count) != count) return false;
    for (uint8_t i = 0; i < count; i++) out[i] = (uint8_t)wire.read();
    return true;
}

int16_t bigEndian16(const uint8_t* data)
{
    return (int16_t)((data[0] << 8) | data[1]);
}

// Chip axes are x = arrow, y = left, z = up out of the chip. Converts to the body
// frame (x forward, y right, z down), then applies the mounting from config.h.
void chipToBody(const float chip[AXIS_COUNT], float body[AXIS_COUNT])
{
    float x = chip[0];
    float y = -chip[1];
    float z = -chip[2];

    float rotatedX = x;
    float rotatedY = y;
    switch (IMU_ALIGN_YAW_DEG) {
    case 90:  rotatedX = -y; rotatedY = x;  break;
    case 180: rotatedX = -x; rotatedY = -y; break;
    case 270: rotatedX = y;  rotatedY = -x; break;
    default: break;
    }

    if (IMU_UPSIDE_DOWN) {
        // Flipped about the forward axis: right and down swap sign.
        rotatedY = -rotatedY;
        z = -z;
    }
    body[0] = rotatedX;
    body[1] = rotatedY;
    body[2] = z;
}

// Raw read converted to body-frame units, no bias or filtering.
bool readBody(float gyroDps[AXIS_COUNT], float accelG[AXIS_COUNT])
{
    uint8_t data[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, data, sizeof(data))) return false;

    float chipAccel[AXIS_COUNT];
    float chipGyro[AXIS_COUNT];
    for (int i = 0; i < AXIS_COUNT; i++) {
        chipAccel[i] = bigEndian16(&data[i * 2]) / ACCEL_LSB_PER_G;
        // Bytes 6..7 are temperature, gyro starts at 8.
        chipGyro[i] = bigEndian16(&data[8 + i * 2]) / GYRO_LSB_PER_DPS;
    }
    chipToBody(chipGyro, gyroDps);
    chipToBody(chipAccel, accelG);
    return true;
}

}  // namespace

namespace imu {

bool init()
{
    // Reset first so a warm reboot starts from known register values.
    writeRegister(REG_PWR_MGMT_1, 0x80);
    delay(100);

    uint8_t id = 0;
    if (!readRegisters(REG_WHO_AM_I, &id, 1)) return false;
    bool known = false;
    for (unsigned i = 0; i < sizeof(KNOWN_IDS); i++) {
        if (id == KNOWN_IDS[i]) known = true;
    }
    if (!known) return false;
    chipId = id;

    bool ok = true;
    ok &= writeRegister(REG_PWR_MGMT_1, 0x01);    // wake up, clock from gyro X PLL
    delay(10);
    ok &= writeRegister(REG_CONFIG, 0x01);        // DLPF 188 Hz, gyro output 1 kHz
    ok &= writeRegister(REG_SMPLRT_DIV, 0x00);    // 1 kHz sample rate
    ok &= writeRegister(REG_GYRO_CONFIG, 0x18);   // +-2000 deg/s
    ok &= writeRegister(REG_ACCEL_CONFIG, 0x10);  // +-8 g
    if (!ok) return false;

    for (int i = 0; i < AXIS_COUNT; i++) {
        sample.gyroDps[i] = 0.0f;
        sample.gyroRawDps[i] = 0.0f;
        sample.accelG[i] = 0.0f;
    }
    sample.accelG[2] = -1.0f;
    isHealthy = true;
    eventLog::post(LogLevel::Info, LogCode::ImuFound, chipId);
    return true;
}

void configure(float gyroLpfHz, float loopHz, const float offsetG[AXIS_COUNT])
{
    for (int i = 0; i < AXIS_COUNT; i++) {
        gyroFilters[i].configure(gyroLpfHz, loopHz);
        accelOffsetG[i] = offsetG[i];
    }
}

bool calibrateGyro()
{
    float sum[AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
    float minimum[AXIS_COUNT];
    float maximum[AXIS_COUNT];
    uint32_t good = 0;

    for (uint32_t n = 0; n < GYRO_CAL_SAMPLES; n++) {
        float gyro[AXIS_COUNT];
        float accel[AXIS_COUNT];
        if (readBody(gyro, accel)) {
            for (int i = 0; i < AXIS_COUNT; i++) {
                if (good == 0 || gyro[i] < minimum[i]) minimum[i] = gyro[i];
                if (good == 0 || gyro[i] > maximum[i]) maximum[i] = gyro[i];
                sum[i] += gyro[i];
            }
            good++;
        }
        board::ledWrite((n / 50) % 2 == 0);
        delayMicroseconds(1000);
    }
    board::ledWrite(false);

    // Most reads must succeed, a flaky bus shouldn't produce a bias from a few samples.
    if (good < GYRO_CAL_SAMPLES * 9 / 10) return false;
    for (int i = 0; i < AXIS_COUNT; i++) {
        if (maximum[i] - minimum[i] > GYRO_CAL_MAX_SPREAD_DPS) return false;
    }
    for (int i = 0; i < AXIS_COUNT; i++) {
        gyroBiasDps[i] = sum[i] / good;
        gyroFilters[i].reset(0.0f);
    }
    calibrated = true;
    return true;
}

bool calibrateAccel(float offsetOutG[AXIS_COUNT])
{
    const uint32_t samples = 500;
    float sum[AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
    uint32_t good = 0;
    for (uint32_t n = 0; n < samples; n++) {
        float gyro[AXIS_COUNT];
        float accel[AXIS_COUNT];
        if (readBody(gyro, accel)) {
            for (int i = 0; i < AXIS_COUNT; i++) sum[i] += accel[i];
            good++;
        }
        board::watchdogKick();
        delayMicroseconds(1000);
    }
    if (good < samples * 9 / 10) return false;

    // Level and still, the reading should be exactly (0, 0, -1) g.
    const float expected[AXIS_COUNT] = { 0.0f, 0.0f, -1.0f };
    for (int i = 0; i < AXIS_COUNT; i++) {
        offsetOutG[i] = sum[i] / good - expected[i];
        if (offsetOutG[i] < -0.5f || offsetOutG[i] > 0.5f) return false;
    }
    return true;
}

const ImuSample& read()
{
    float gyro[AXIS_COUNT];
    float accel[AXIS_COUNT];
    if (!readBody(gyro, accel)) {
        totalErrors++;
        consecutiveErrors++;
        if (isHealthy && consecutiveErrors >= IMU_MAX_CONSECUTIVE_ERRORS) {
            isHealthy = false;
            eventLog::post(LogLevel::Error, LogCode::ImuFault, (int32_t)consecutiveErrors);
        }
        return sample;
    }
    consecutiveErrors = 0;

    for (int i = 0; i < AXIS_COUNT; i++) {
        sample.gyroRawDps[i] = gyro[i] - gyroBiasDps[i];
        sample.gyroDps[i] = gyroFilters[i].apply(sample.gyroRawDps[i]);
        sample.accelG[i] = accel[i] - accelOffsetG[i];
    }
    return sample;
}

bool healthy()
{
    return isHealthy;
}

bool gyroCalibrated()
{
    return calibrated;
}

uint32_t errorCount()
{
    return totalErrors;
}

uint8_t whoAmI()
{
    return chipId;
}

}  // namespace imu
