#include "unity.h"
#include "PIDController.hpp"
#include "control_math/WheelVelocityController.hpp"
#include <cmath>

namespace {
control_math::PidParameters makeParameters()
{
    return {
        0.01f,
        0.05f,
        0.0001f,
        -1.0f,
        1.0f,
        -10.0f,
        10.0f
    };
}

PIDConfig makeLegacyConfig()
{
    return {
        0.01f,
        0.05f,
        0.0001f,
        -1.0f,
        1.0f,
        -10.0f,
        10.0f
    };
}
}

TEST_CASE("portable wheel velocity controller preserves the legacy PID response", "[control][pid]")
{
    control_math::WheelVelocityController wheel(makeParameters());
    PIDController legacy("portable-compatibility");
    TEST_ASSERT_EQUAL(ESP_OK, legacy.init(makeLegacyConfig()));

    const float targets[] = {100.0f, 100.0f, 80.0f};
    const float measurements[] = {0.0f, 50.0f, 70.0f};
    const float periods[] = {0.005f, 0.005f, 0.010f};
    const float expected[] = {1.0f, -0.46875f, -0.25375f};

    for (size_t i = 0; i < 3; ++i) {
        const auto result = wheel.update(targets[i], measurements[i], periods[i]);
        const float legacyOutput = legacy.compute(targets[i], measurements[i], periods[i]);
        TEST_ASSERT_TRUE(result.valid);
        TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected[i], result.effort);
        TEST_ASSERT_FLOAT_WITHIN(0.000001f, legacyOutput, result.effort);
        TEST_ASSERT_EQUAL(i == 0, result.saturated);
    }
}

TEST_CASE("portable wheel velocity controller supports measurement-rate derivative and reset", "[control][pid]")
{
    control_math::PidParameters parameters = makeParameters();
    parameters.kp = 0.0f;
    parameters.ki = 0.0f;
    parameters.kd = 0.01f;
    control_math::WheelVelocityController wheel(parameters);

    const auto first = wheel.update(0.0f, 0.0f, 0.005f);
    TEST_ASSERT_TRUE(first.valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, first.effort);

    const auto second = wheel.updateWithMeasurementRate(0.0f, 20.0f, 20.0f, 0.005f);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, -0.2f, second.effort);

    wheel.reset();
    const auto afterReset = wheel.updateWithMeasurementRate(0.0f, 20.0f, 20.0f, 0.005f);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, -0.2f, afterReset.effort);
}

TEST_CASE("portable wheel velocity controller rejects invalid input without producing effort", "[control][pid]")
{
    control_math::WheelVelocityController wheel(makeParameters());
    const auto invalid = wheel.update(NAN, 0.0f, 0.005f);
    TEST_ASSERT_FALSE(invalid.valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, invalid.effort);
}
