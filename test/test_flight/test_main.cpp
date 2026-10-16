#include <unity.h>
#include <math.h>

#include "angle_mode.h"
#include "attitude.h"
#include "mixer.h"
#include "pid.h"
#include "pt1_filter.h"
#include "rate_controller.h"
#include "rates.h"
#include "runaway_detector.h"

void setUp() {}
void tearDown() {}

static RcInput centeredSticks()
{
    RcInput rc;
    for (int axis = 0; axis < AXIS_COUNT; axis++) rc.stick[axis] = 0.0f;
    rc.throttle = 0.0f;
    for (int i = 0; i < RC_AUX_COUNT; i++) rc.aux[i] = SwitchPos::Low;
    return rc;
}

static void test_pt1_reaches_63_percent_after_time_constant()
{
    Pt1Filter filter;
    float cutoffHz = 10.0f;
    float sampleHz = 10000.0f;
    filter.configure(cutoffHz, sampleHz);
    int samples = (int)(sampleHz / (2.0f * 3.14159265f * cutoffHz));
    for (int i = 0; i < samples; i++) filter.apply(1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.632f, filter.value());
}

static void test_pt1_zero_cutoff_passes_through()
{
    Pt1Filter filter;
    filter.configure(0.0f, 1000.0f);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, filter.apply(5.0f));
}

static void test_pid_p_term()
{
    Pid pid;
    PidGains gains = { 0.01f, 0.0f, 0.0f, 0.3f };
    pid.configure(gains, 0.0f, 1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, pid.update(100.0f, 50.0f, 0.001f, true));
}

static void test_pid_integral_is_clamped()
{
    Pid pid;
    PidGains gains = { 0.0f, 1.0f, 0.0f, 0.3f };
    pid.configure(gains, 0.0f, 1000.0f);
    for (int i = 0; i < 1000; i++) pid.update(100.0f, 0.0f, 0.001f, true);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, pid.terms().i);
}

static void test_pid_integral_frozen_when_not_integrating()
{
    Pid pid;
    PidGains gains = { 0.0f, 1.0f, 0.0f, 0.3f };
    pid.configure(gains, 0.0f, 1000.0f);
    pid.update(10.0f, 0.0f, 0.001f, false);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.terms().i);
}

static void test_pid_d_on_measurement_ignores_setpoint_step()
{
    Pid pid;
    PidGains gains = { 0.0f, 0.0f, 0.001f, 0.3f };
    pid.configure(gains, 0.0f, 1000.0f);
    pid.update(0.0f, 0.0f, 0.001f, true);
    pid.update(500.0f, 0.0f, 0.001f, true);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.terms().d);
    // Measured rate rising opposes the motion: negative D.
    pid.update(500.0f, 10.0f, 0.001f, true);
    TEST_ASSERT_TRUE(pid.terms().d < 0.0f);
}

static void test_pid_output_clamped()
{
    Pid pid;
    PidGains gains = { 1.0f, 0.0f, 0.0f, 0.3f };
    pid.configure(gains, 0.0f, 1000.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.update(1000.0f, 0.0f, 0.001f, true));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, pid.update(-1000.0f, 0.0f, 0.001f, true));
}

static void test_pid_nan_is_discarded()
{
    Pid pid;
    PidGains gains = { 0.01f, 1.0f, 0.0f, 0.3f };
    pid.configure(gains, 0.0f, 1000.0f);
    pid.update(10.0f, 0.0f, 0.001f, true);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(NAN, 0.0f, 0.001f, true));
    TEST_ASSERT_EQUAL_UINT32(1, pid.faults());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.terms().i);
    TEST_ASSERT_FALSE(isnan(pid.update(10.0f, 0.0f, 0.001f, true)));
}

static void test_rate_controller_reports_saturation()
{
    RateController controller;
    RateControllerConfig config;
    for (int axis = 0; axis < AXIS_COUNT; axis++) {
        PidGains gains = { 0.01f, 0.0f, 0.0f, 0.3f };
        config.gains[axis] = gains;
    }
    config.dtermCutoffHz = 0.0f;
    config.loopHz = 1000.0f;
    controller.configure(config);

    RateSetpoint target = { { 500.0f, 0.0f, 0.0f } };
    float gyro[AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
    controller.update(target, gyro, 0.001f, true);
    TEST_ASSERT_TRUE(controller.saturated());
    TEST_ASSERT_EQUAL_FLOAT(500.0f, controller.maxAbsErrorDps());
}

static void test_rates_linear_and_expo()
{
    TEST_ASSERT_EQUAL_FLOAT(250.0f, stickToRate(0.5f, 500.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(500.0f, stickToRate(1.0f, 500.0f, 0.5f));
    TEST_ASSERT_TRUE(stickToRate(0.5f, 500.0f, 0.5f) < 250.0f);
    TEST_ASSERT_EQUAL_FLOAT(-500.0f, stickToRate(-2.0f, 500.0f, 0.0f));
}

static void test_mixer_hover_is_even()
{
    MixerConfig config = { false, 0.0f };
    AxisTorque torque = { { 0.0f, 0.0f, 0.0f } };
    MotorOutputs out = mixerMix(torque, 0.5f, config);
    for (int i = 0; i < MOTOR_COUNT; i++) TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, out.motor[i]);
}

static void test_mixer_roll_right_raises_left_motors()
{
    MixerConfig config = { false, 0.0f };
    AxisTorque torque = { { 0.1f, 0.0f, 0.0f } };
    MotorOutputs out = mixerMix(torque, 0.5f, config);
    TEST_ASSERT_TRUE(out.motor[2] > 0.5f);  // rear-left
    TEST_ASSERT_TRUE(out.motor[3] > 0.5f);  // front-left
    TEST_ASSERT_TRUE(out.motor[0] < 0.5f);  // rear-right
    TEST_ASSERT_TRUE(out.motor[1] < 0.5f);  // front-right
}

static void test_mixer_pitch_up_raises_front_motors()
{
    MixerConfig config = { false, 0.0f };
    AxisTorque torque = { { 0.0f, 0.1f, 0.0f } };
    MotorOutputs out = mixerMix(torque, 0.5f, config);
    TEST_ASSERT_TRUE(out.motor[1] > 0.5f);
    TEST_ASSERT_TRUE(out.motor[3] > 0.5f);
    TEST_ASSERT_TRUE(out.motor[0] < 0.5f);
    TEST_ASSERT_TRUE(out.motor[2] < 0.5f);
}

static void test_mixer_yaw_reversed_flips_yaw()
{
    MixerConfig normal = { false, 0.0f };
    MixerConfig reversed = { true, 0.0f };
    AxisTorque torque = { { 0.0f, 0.0f, 0.1f } };
    MotorOutputs a = mixerMix(torque, 0.5f, normal);
    MotorOutputs b = mixerMix(torque, 0.5f, reversed);
    TEST_ASSERT_TRUE(a.motor[1] > 0.5f);
    TEST_ASSERT_TRUE(b.motor[1] < 0.5f);
}

static void test_mixer_keeps_attitude_authority_at_zero_throttle()
{
    MixerConfig config = { false, 0.0f };
    AxisTorque torque = { { 0.2f, 0.0f, 0.0f } };
    MotorOutputs out = mixerMix(torque, 0.0f, config);
    // Throttle is shifted up so the roll difference survives instead of clipping at 0.
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, out.motor[2] - out.motor[0]);
    for (int i = 0; i < MOTOR_COUNT; i++) TEST_ASSERT_TRUE(out.motor[i] >= 0.0f);
}

static void test_mixer_desaturates_large_commands()
{
    MixerConfig config = { false, 0.0f };
    AxisTorque torque = { { 1.0f, 1.0f, 1.0f } };
    MotorOutputs out = mixerMix(torque, 0.5f, config);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        TEST_ASSERT_TRUE(out.motor[i] >= 0.0f);
        TEST_ASSERT_TRUE(out.motor[i] <= 1.0f);
    }
    // Front-left gets +roll +pitch -yaw, the most of all four.
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.motor[3] > out.motor[0] ? 1.0f : 0.0f);
}

static void test_mixer_idle_offset()
{
    MixerConfig config = { false, 0.05f };
    AxisTorque torque = { { 0.0f, 0.0f, 0.0f } };
    MotorOutputs out = mixerMix(torque, 0.0f, config);
    for (int i = 0; i < MOTOR_COUNT; i++) TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, out.motor[i]);
}

static void test_accel_angles()
{
    float level[3] = { 0.0f, 0.0f, -1.0f };
    Angles a = anglesFromAccel(level);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, a.rollDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, a.pitchDeg);

    // Nose up 30 deg: gravity reaction gets a forward component.
    float noseUp[3] = { 0.5f, 0.0f, -0.866f };
    a = anglesFromAccel(noseUp);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, a.pitchDeg);

    // Right side down 30 deg: reaction points partly to the left (-y).
    float rollRight[3] = { 0.0f, -0.5f, -0.866f };
    a = anglesFromAccel(rollRight);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, a.rollDeg);
}

static void test_attitude_integrates_gyro()
{
    AttitudeEstimator estimator;
    estimator.configure(1000.0f);  // almost no accel correction
    float level[3] = { 0.0f, 0.0f, -1.0f };
    estimator.resetFromAccel(level);

    ImuSample imu;
    imu.gyroDps[AXIS_ROLL] = 90.0f;
    imu.gyroDps[AXIS_PITCH] = 0.0f;
    imu.gyroDps[AXIS_YAW] = 0.0f;
    imu.accelG[0] = 0.0f;
    imu.accelG[1] = 0.0f;
    imu.accelG[2] = 0.0f;  // free fall, accel ignored
    for (int i = 0; i < 100; i++) estimator.update(imu, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 9.0f, estimator.angles().rollDeg);
}

static void test_attitude_accel_pulls_back()
{
    AttitudeEstimator estimator;
    estimator.configure(0.5f);
    ImuSample imu;
    for (int axis = 0; axis < AXIS_COUNT; axis++) imu.gyroDps[axis] = 0.0f;
    imu.accelG[0] = 0.5f;
    imu.accelG[1] = 0.0f;
    imu.accelG[2] = -0.866f;
    for (int i = 0; i < 5000; i++) estimator.update(imu, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, estimator.angles().pitchDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, estimator.tiltDeg());
}

static void test_angle_mode_levels()
{
    AngleModeConfig config = { 45.0f, 5.0f, 300.0f };
    RatesConfig rates = { { 500.0f, 500.0f, 400.0f }, 0.0f };
    RcInput rc = centeredSticks();
    Angles tilted = { 10.0f, -4.0f };
    RateSetpoint target = angleModeSetpoint(rc, tilted, config, rates);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -50.0f, target.dps[AXIS_ROLL]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 20.0f, target.dps[AXIS_PITCH]);

    rc.stick[AXIS_ROLL] = 1.0f;
    Angles level = { 0.0f, 0.0f };
    target = angleModeSetpoint(rc, level, config, rates);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 225.0f, target.dps[AXIS_ROLL]);

    rc.stick[AXIS_YAW] = 0.5f;
    target = angleModeSetpoint(rc, level, config, rates);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 200.0f, target.dps[AXIS_YAW]);
}

static void test_runaway_trips_after_time()
{
    RunawayDetector detector;
    detector.configure(200.0f, 1000);
    TEST_ASSERT_FALSE(detector.update(true, 500.0f, true, 0));
    TEST_ASSERT_FALSE(detector.update(true, 500.0f, true, 999));
    TEST_ASSERT_TRUE(detector.update(true, 500.0f, true, 1000));
}

static void test_runaway_resets_when_recovered()
{
    RunawayDetector detector;
    detector.configure(200.0f, 1000);
    detector.update(true, 500.0f, true, 0);
    detector.update(true, 50.0f, true, 500);
    TEST_ASSERT_FALSE(detector.update(true, 500.0f, true, 1200));
    TEST_ASSERT_FALSE(detector.update(false, 500.0f, true, 3000));
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_pt1_reaches_63_percent_after_time_constant);
    RUN_TEST(test_pt1_zero_cutoff_passes_through);
    RUN_TEST(test_pid_p_term);
    RUN_TEST(test_pid_integral_is_clamped);
    RUN_TEST(test_pid_integral_frozen_when_not_integrating);
    RUN_TEST(test_pid_d_on_measurement_ignores_setpoint_step);
    RUN_TEST(test_pid_output_clamped);
    RUN_TEST(test_pid_nan_is_discarded);
    RUN_TEST(test_rate_controller_reports_saturation);
    RUN_TEST(test_rates_linear_and_expo);
    RUN_TEST(test_mixer_hover_is_even);
    RUN_TEST(test_mixer_roll_right_raises_left_motors);
    RUN_TEST(test_mixer_pitch_up_raises_front_motors);
    RUN_TEST(test_mixer_yaw_reversed_flips_yaw);
    RUN_TEST(test_mixer_keeps_attitude_authority_at_zero_throttle);
    RUN_TEST(test_mixer_desaturates_large_commands);
    RUN_TEST(test_mixer_idle_offset);
    RUN_TEST(test_accel_angles);
    RUN_TEST(test_attitude_integrates_gyro);
    RUN_TEST(test_attitude_accel_pulls_back);
    RUN_TEST(test_angle_mode_levels);
    RUN_TEST(test_runaway_trips_after_time);
    RUN_TEST(test_runaway_resets_when_recovered);
    return UNITY_END();
}
