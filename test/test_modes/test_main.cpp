#include <unity.h>

#include "mode_manager.h"

void setUp() {}
void tearDown() {}

static ModeConfig testConfig()
{
    ModeConfig config;
    config.armThrottleMax = 0.05f;
    config.armMaxTiltDeg = 25.0f;
    config.bootGraceMs = 2000;
    config.failsafeStage1Ms = 1000;
    config.failsafeThrottle = 0.0f;
    return config;
}

// Healthy, linked, switch off, past the boot grace period.
static ModeInputs readyInputs(uint32_t nowMs)
{
    ModeInputs in;
    in.nowMs = nowMs;
    for (int axis = 0; axis < AXIS_COUNT; axis++) in.rc.stick[axis] = 0.0f;
    in.rc.throttle = 0.0f;
    for (int i = 0; i < RC_AUX_COUNT; i++) in.rc.aux[i] = SwitchPos::Low;
    in.linkOk = true;
    in.gyroCalibrated = true;
    in.imuHealthy = true;
    in.tiltDeg = 0.0f;
    in.batteryCritical = false;
    in.cliActive = false;
    in.loopOverloaded = false;
    in.runawayDetected = false;
    return in;
}

static ModeEvent setArmSwitch(ModeManager& modes, ModeInputs& in, bool on)
{
    in.rc.aux[AUX_ARM] = on ? SwitchPos::High : SwitchPos::Low;
    return modes.update(in);
}

// Brings a fresh manager to the armed state at t = 3000 ms.
static ModeInputs armedManager(ModeManager& modes)
{
    modes.configure(testConfig());
    ModeInputs in = readyInputs(3000);
    modes.update(in);
    setArmSwitch(modes, in, true);
    return in;
}

static void test_arms_on_switch_edge_when_ready()
{
    ModeManager modes;
    modes.configure(testConfig());
    ModeInputs in = readyInputs(3000);
    // The first linked update reports the initial flight mode switch position.
    TEST_ASSERT_EQUAL(ModeEvent::FlightModeChanged, modes.update(in));
    TEST_ASSERT_EQUAL(ModeEvent::None, modes.update(in));
    TEST_ASSERT_EQUAL_UINT16(0, modes.armBlockFlags());
    TEST_ASSERT_EQUAL(ModeEvent::Armed, setArmSwitch(modes, in, true));
    TEST_ASSERT_TRUE(modes.isArmed());
}

static void test_switch_on_at_boot_never_arms()
{
    ModeManager modes;
    modes.configure(testConfig());
    ModeInputs in = readyInputs(3000);
    in.rc.aux[AUX_ARM] = SwitchPos::High;
    TEST_ASSERT_EQUAL(ModeEvent::ArmBlocked, modes.update(in));
    TEST_ASSERT_FALSE(modes.isArmed());
    TEST_ASSERT_TRUE(modes.armBlockFlags() & ARM_BLOCK_SWITCH_NOT_CYCLED);
    in.nowMs += 100;
    TEST_ASSERT_EQUAL(ModeEvent::None, modes.update(in));
    TEST_ASSERT_FALSE(modes.isArmed());
}

static void test_each_block_flag_prevents_arming()
{
    struct Case {
        uint16_t flag;
        void (*apply)(ModeInputs&);
    };
    struct Apply {
        static void throttle(ModeInputs& in) { in.rc.throttle = 0.5f; }
        static void calibration(ModeInputs& in) { in.gyroCalibrated = false; }
        static void imu(ModeInputs& in) { in.imuHealthy = false; }
        static void tilt(ModeInputs& in) { in.tiltDeg = 40.0f; }
        static void cli(ModeInputs& in) { in.cliActive = true; }
        static void battery(ModeInputs& in) { in.batteryCritical = true; }
        static void overload(ModeInputs& in) { in.loopOverloaded = true; }
        static void boot(ModeInputs& in) { in.nowMs = 500; }
    };
    const Case cases[] = {
        { ARM_BLOCK_THROTTLE_HIGH, Apply::throttle },
        { ARM_BLOCK_NOT_CALIBRATED, Apply::calibration },
        { ARM_BLOCK_IMU_FAULT, Apply::imu },
        { ARM_BLOCK_TILTED, Apply::tilt },
        { ARM_BLOCK_CLI_ACTIVE, Apply::cli },
        { ARM_BLOCK_BATTERY_CRITICAL, Apply::battery },
        { ARM_BLOCK_LOOP_OVERLOAD, Apply::overload },
        { ARM_BLOCK_BOOT_GRACE, Apply::boot },
    };

    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        ModeManager modes;
        modes.configure(testConfig());
        ModeInputs in = readyInputs(3000);
        cases[i].apply(in);
        modes.update(in);
        TEST_ASSERT_TRUE(modes.armBlockFlags() & cases[i].flag);
        TEST_ASSERT_EQUAL(ModeEvent::ArmBlocked, setArmSwitch(modes, in, true));
        TEST_ASSERT_FALSE(modes.isArmed());
    }
}

static void test_no_link_blocks_arming()
{
    ModeManager modes;
    modes.configure(testConfig());
    ModeInputs in = readyInputs(3000);
    in.linkOk = false;
    modes.update(in);
    TEST_ASSERT_TRUE(modes.armBlockFlags() & ARM_BLOCK_NO_LINK);
    // Without a link the switch reads as off, so no arm attempt happens at all.
    TEST_ASSERT_EQUAL(ModeEvent::None, setArmSwitch(modes, in, true));
    TEST_ASSERT_FALSE(modes.isArmed());
}

static void test_blocked_attempt_needs_new_switch_cycle()
{
    ModeManager modes;
    modes.configure(testConfig());
    ModeInputs in = readyInputs(3000);
    in.rc.throttle = 0.5f;
    modes.update(in);
    TEST_ASSERT_EQUAL(ModeEvent::ArmBlocked, setArmSwitch(modes, in, true));

    // Throttle comes down with the switch still on: must stay disarmed.
    in.rc.throttle = 0.0f;
    TEST_ASSERT_EQUAL(ModeEvent::None, modes.update(in));
    TEST_ASSERT_FALSE(modes.isArmed());

    setArmSwitch(modes, in, false);
    TEST_ASSERT_EQUAL(ModeEvent::Armed, setArmSwitch(modes, in, true));
}

static void test_disarms_on_switch_off()
{
    ModeManager modes;
    ModeInputs in = armedManager(modes);
    TEST_ASSERT_EQUAL(ModeEvent::Disarmed, setArmSwitch(modes, in, false));
    TEST_ASSERT_EQUAL(DisarmReason::Switch, modes.lastDisarmReason());
}

static void test_throttle_follows_state()
{
    ModeManager modes;
    ModeInputs in = armedManager(modes);
    in.rc.throttle = 0.4f;
    modes.update(in);
    TEST_ASSERT_EQUAL_FLOAT(0.4f, modes.throttle());
    setArmSwitch(modes, in, false);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, modes.throttle());
}

static void test_failsafe_timeline_disarms_after_stage1()
{
    ModeManager modes;
    ModeInputs in = armedManager(modes);

    in.linkOk = false;
    in.nowMs = 4000;
    TEST_ASSERT_EQUAL(ModeEvent::FailsafeStarted, modes.update(in));
    TEST_ASSERT_TRUE(modes.inFailsafe());
    TEST_ASSERT_TRUE(modes.isArmed());
    TEST_ASSERT_EQUAL(FlightMode::Angle, modes.flightMode());

    in.nowMs = 4999;
    TEST_ASSERT_EQUAL(ModeEvent::None, modes.update(in));
    in.nowMs = 5000;
    TEST_ASSERT_EQUAL(ModeEvent::Disarmed, modes.update(in));
    TEST_ASSERT_EQUAL(DisarmReason::Failsafe, modes.lastDisarmReason());

    // Link comes back with the switch still on: no re-arm without a switch cycle.
    in.linkOk = true;
    in.nowMs = 6000;
    modes.update(in);
    TEST_ASSERT_FALSE(modes.isArmed());
    TEST_ASSERT_TRUE(modes.armBlockFlags() & ARM_BLOCK_SWITCH_NOT_CYCLED);
}

static void test_failsafe_recovers_when_link_returns()
{
    ModeManager modes;
    ModeInputs in = armedManager(modes);
    in.linkOk = false;
    in.nowMs = 4000;
    modes.update(in);
    in.linkOk = true;
    in.nowMs = 4500;
    TEST_ASSERT_EQUAL(ModeEvent::FailsafeRecovered, modes.update(in));
    TEST_ASSERT_EQUAL(ArmState::Armed, modes.armState());
}

static void test_failsafe_uses_failsafe_throttle()
{
    ModeManager modes;
    ModeConfig config = testConfig();
    config.failsafeThrottle = 0.3f;
    modes.configure(config);
    ModeInputs in = readyInputs(3000);
    modes.update(in);
    setArmSwitch(modes, in, true);
    in.rc.throttle = 0.6f;
    in.linkOk = false;
    modes.update(in);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, modes.throttle());
}

static void test_imu_fault_disarms_immediately()
{
    ModeManager modes;
    ModeInputs in = armedManager(modes);
    in.imuHealthy = false;
    TEST_ASSERT_EQUAL(ModeEvent::Disarmed, modes.update(in));
    TEST_ASSERT_EQUAL(DisarmReason::ImuFault, modes.lastDisarmReason());
}

static void test_runaway_disarms()
{
    ModeManager modes;
    ModeInputs in = armedManager(modes);
    in.runawayDetected = true;
    TEST_ASSERT_EQUAL(ModeEvent::Disarmed, modes.update(in));
    TEST_ASSERT_EQUAL(DisarmReason::Runaway, modes.lastDisarmReason());
}

static void test_flight_mode_switch()
{
    ModeManager modes;
    modes.configure(testConfig());
    ModeInputs in = readyInputs(3000);
    in.rc.aux[AUX_FLIGHT_MODE] = SwitchPos::Low;
    modes.update(in);
    TEST_ASSERT_EQUAL(FlightMode::Acro, modes.flightMode());
    TEST_ASSERT_EQUAL_STRING("ACRO", modes.flightModeName());
    in.rc.aux[AUX_FLIGHT_MODE] = SwitchPos::High;
    TEST_ASSERT_EQUAL(ModeEvent::FlightModeChanged, modes.update(in));
    TEST_ASSERT_EQUAL(FlightMode::Angle, modes.flightMode());
}

static void test_flag_names()
{
    TEST_ASSERT_EQUAL_STRING("NO_LINK", ModeManager::armBlockFlagName(0));
    TEST_ASSERT_EQUAL_STRING("BOOT_GRACE", ModeManager::armBlockFlagName(9));
    TEST_ASSERT_EQUAL_STRING("?", ModeManager::armBlockFlagName(10));
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_arms_on_switch_edge_when_ready);
    RUN_TEST(test_switch_on_at_boot_never_arms);
    RUN_TEST(test_each_block_flag_prevents_arming);
    RUN_TEST(test_no_link_blocks_arming);
    RUN_TEST(test_blocked_attempt_needs_new_switch_cycle);
    RUN_TEST(test_disarms_on_switch_off);
    RUN_TEST(test_throttle_follows_state);
    RUN_TEST(test_failsafe_timeline_disarms_after_stage1);
    RUN_TEST(test_failsafe_recovers_when_link_returns);
    RUN_TEST(test_failsafe_uses_failsafe_throttle);
    RUN_TEST(test_imu_fault_disarms_immediately);
    RUN_TEST(test_runaway_disarms);
    RUN_TEST(test_flight_mode_switch);
    RUN_TEST(test_flag_names);
    return UNITY_END();
}
