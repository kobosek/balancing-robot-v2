#include "unity.h"
#include "ControlLoopTiming.hpp"
#include "EncoderService.hpp"
#include "PIDController.hpp"
#include "pcnt_fakes.hpp"
#include "sensor_fakes.hpp"
#include <cmath>

TEST_CASE("late control wake rebases the next deadline instead of catching up", "[control][timing]") {
    ControlLoopTiming timing(1000, 1000000);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.005f, timing.beginStep(1005, 1005000));
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.017f, timing.beginStep(1022, 1022000));
    TEST_ASSERT_EQUAL_UINT32(1027, timing.wakeTick() + 5);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.005f, timing.beginStep(1027, 1027000));
    // Old code substituted 5 ms after >25 ms, corrupting I/D timing.
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.080f, timing.beginStep(1107, 1107000));
    TEST_ASSERT_EQUAL_UINT32(1112, timing.wakeTick() + 5);
}
TEST_CASE("control elapsed time remains measured across RTOS tick wrap", "[control][timing]") {
    ControlLoopTiming timing(static_cast<TickType_t>(-3), 1000000);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.005f, timing.beginStep(2, 1005000));
    TEST_ASSERT_EQUAL_UINT32(7, timing.wakeTick() + 5);
}
TEST_CASE("encoder jitter does not cause the old amplified derivative impulse at constant setpoint", "[control][encoder]") {
    pcnt_fake::reset(); sensor_fake::clockUs = 1000000;
    EncoderService encoders(EncoderConfig{}, 5000); TEST_ASSERT_EQUAL(ESP_OK, encoders.init());
    PIDConfig config; config.pid_kd = 0.0001f; // Synthetic test gain, not a firmware default.
    PIDController pid("timing-test"); TEST_ASSERT_EQUAL(ESP_OK, pid.init(config));
    pid.compute(0, 0, 0.005f);
    sensor_fake::clockUs += 1000; pcnt_fake::advance(0, 1); encoders.update();
    const auto speed = encoders.getFrame().left.speedDps;
    const float effort = pid.compute(0, speed, 0.001f);
    sensor_fake::clockUs = -1;
    // The old fixed-alpha single-pulse response saturates the same controller.
    pid.reset(); pid.compute(0, 0, 0.005f);
    const float oldSpeed = 0.1f * (360.0f / 2800) / 0.001f;
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, pid.compute(0, oldSpeed, 0.001f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.268f, effort);
}
