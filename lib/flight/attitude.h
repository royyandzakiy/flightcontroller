// Roll and pitch estimate from a complementary filter: the gyro is trusted short term,
// the accelerometer (gravity direction) slowly pulls the estimate back.
#pragma once

#include "flight_types.h"

class AttitudeEstimator {
public:
    AttitudeEstimator();

    // How fast the accelerometer corrects gyro drift. Larger = smoother but slower.
    void configure(float accelTimeConstantS);

    // Sets the angles straight from gravity, use when the quad is still.
    void resetFromAccel(const float accelG[AXIS_COUNT]);

    void update(const ImuSample& imu, float dt);

    Angles angles() const;
    // Angle between the body z axis and vertical, 0 when level.
    float tiltDeg() const;

private:
    float rollDeg_;
    float pitchDeg_;
    float timeConstantS_;
};

// Roll and pitch implied by gravity alone. Only valid when not accelerating.
Angles anglesFromAccel(const float accelG[AXIS_COUNT]);
