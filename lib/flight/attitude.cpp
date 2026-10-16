#include "attitude.h"

#include <math.h>
#include "fc_math.h"

// Accel magnitude outside this band means the quad is accelerating,
// so the reading no longer points at gravity and is skipped.
static const float ACCEL_TRUST_MIN_G = 0.85f;
static const float ACCEL_TRUST_MAX_G = 1.15f;

Angles anglesFromAccel(const float accelG[AXIS_COUNT])
{
    // At rest the accelerometer measures the reaction to gravity, (0, 0, -1) g when level.
    float x = accelG[0];
    float y = accelG[1];
    float z = accelG[2];
    Angles angles;
    angles.rollDeg = atan2f(-y, -z) * FC_RAD_TO_DEG;
    angles.pitchDeg = atan2f(x, sqrtf(y * y + z * z)) * FC_RAD_TO_DEG;
    return angles;
}

AttitudeEstimator::AttitudeEstimator()
    : rollDeg_(0.0f), pitchDeg_(0.0f), timeConstantS_(2.0f)
{
}

void AttitudeEstimator::configure(float accelTimeConstantS)
{
    timeConstantS_ = (accelTimeConstantS > 0.01f) ? accelTimeConstantS : 0.01f;
}

void AttitudeEstimator::resetFromAccel(const float accelG[AXIS_COUNT])
{
    Angles angles = anglesFromAccel(accelG);
    rollDeg_ = angles.rollDeg;
    pitchDeg_ = angles.pitchDeg;
}

void AttitudeEstimator::update(const ImuSample& imu, float dt)
{
    // Body rates are integrated as angle rates. Close enough for the small angles
    // angle mode works with, a full quaternion filter would be the upgrade.
    rollDeg_ += imu.gyroDps[AXIS_ROLL] * dt;
    pitchDeg_ += imu.gyroDps[AXIS_PITCH] * dt;

    float ax = imu.accelG[0];
    float ay = imu.accelG[1];
    float az = imu.accelG[2];
    float magnitude = sqrtf(ax * ax + ay * ay + az * az);
    if (magnitude >= ACCEL_TRUST_MIN_G && magnitude <= ACCEL_TRUST_MAX_G) {
        Angles fromAccel = anglesFromAccel(imu.accelG);
        float weight = dt / (timeConstantS_ + dt);
        rollDeg_ += weight * (fromAccel.rollDeg - rollDeg_);
        pitchDeg_ += weight * (fromAccel.pitchDeg - pitchDeg_);
    }

    if (rollDeg_ > 180.0f) rollDeg_ -= 360.0f;
    if (rollDeg_ < -180.0f) rollDeg_ += 360.0f;
    pitchDeg_ = fcClamp(pitchDeg_, -90.0f, 90.0f);
}

Angles AttitudeEstimator::angles() const
{
    Angles angles;
    angles.rollDeg = rollDeg_;
    angles.pitchDeg = pitchDeg_;
    return angles;
}

float AttitudeEstimator::tiltDeg() const
{
    float cosTilt = cosf(rollDeg_ * FC_DEG_TO_RAD) * cosf(pitchDeg_ * FC_DEG_TO_RAD);
    return acosf(fcClamp(cosTilt, -1.0f, 1.0f)) * FC_RAD_TO_DEG;
}
