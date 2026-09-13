#include "unity.h"
#include "PIDController.hpp"
#include "control_math/PidCore.hpp"
#include "control_math/WheelVelocityController.hpp"
#include "control_math/EffortMapping.hpp"
#include <cmath>
#include <cfloat>

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

TEST_CASE("PidCore preview is side effect free and commit applies it once",
          "[control][pid][preview]")
{
    control_math::PidCore pid(makeParameters());
    const auto first = pid.compute(1.0f, 0.0f, 0.1f);
    TEST_ASSERT_TRUE(first.valid);

    const auto preview = pid.preview(2.0f, 0.0f, 0.1f);
    TEST_ASSERT_TRUE(preview.valid);
    const auto previewAgain = pid.preview(2.0f, 0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, preview.output, previewAgain.output);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f,
                             preview.integralTerm,
                             previewAgain.integralTerm);

    control_math::PidCore reference(makeParameters());
    reference.compute(1.0f, 0.0f, 0.1f);
    const auto expected = reference.compute(2.0f, 0.0f, 0.1f);
    const auto committed = pid.compute(2.0f, 0.0f, 0.1f);
    TEST_ASSERT_TRUE(committed.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected.output, preview.output);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected.output, committed.output);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f,
                             expected.integralTerm,
                             committed.integralTerm);
}

TEST_CASE("PidCore integrate false advances derivative history without integral windup",
          "[control][pid][antiwindup]")
{
    control_math::PidParameters parameters = makeParameters();
    parameters.kp = 0.0f;
    parameters.ki = 1.0f;
    parameters.kd = 0.1f;
    parameters.outputMin = -10.0f;
    parameters.outputMax = 10.0f;
    parameters.integralMin = -10.0f;
    parameters.integralMax = 10.0f;

    control_math::PidCore pid(parameters);
    const auto first = pid.compute(1.0f, 0.0f, 0.1f);
    TEST_ASSERT_TRUE(first.valid);
    const auto blocked = pid.compute(1.0f, 0.0f, 0.1f, false);
    TEST_ASSERT_TRUE(blocked.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, first.integralTerm,
                             blocked.integralTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, blocked.derivativeTerm);
    // `integrate=false` is an explicit anti-windup preview/hold step: it
    // advances derivative history but contributes no integral delta.
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, blocked.integralDelta);

    // The following derivative uses the blocked step's error, while the
    // integral still contains only the first step's contribution.
    const auto next = pid.compute(0.0f, 0.0f, 0.1f);
    TEST_ASSERT_TRUE(next.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, -1.0f, next.derivativeTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.10f, next.integralTerm);
}

TEST_CASE("PidCore rejects arithmetic overflow without contaminating state",
          "[control][pid][invalid]")
{
    control_math::PidCore pid(makeParameters());
    const auto invalid = pid.compute(FLT_MAX, -FLT_MAX, 0.1f);
    TEST_ASSERT_FALSE(invalid.valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, invalid.output);

    control_math::PidCore reference(makeParameters());
    const auto expected = reference.compute(0.5f, 0.1f, 0.01f);
    const auto actual = pid.compute(0.5f, 0.1f, 0.01f);
    TEST_ASSERT_TRUE(actual.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected.output, actual.output);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f,
                             expected.integralTerm,
                             actual.integralTerm);
}

TEST_CASE("PidCore rejects invalid parameter ordering before a step",
          "[control][pid][invalid]")
{
    control_math::PidParameters invalid = makeParameters();
    invalid.outputMin = 2.0f;
    invalid.outputMax = -2.0f;
    control_math::PidCore pid(invalid);
    TEST_ASSERT_FALSE(pid.parametersValid());
    TEST_ASSERT_FALSE(pid.compute(0.0f, 0.0f, 0.01f).valid);

    pid.setParameters(makeParameters());
    TEST_ASSERT_TRUE(pid.parametersValid());
    TEST_ASSERT_TRUE(pid.compute(0.0f, 0.0f, 0.01f).valid);
}

TEST_CASE("PidCore rejects invalid reconfiguration without losing active state",
          "[control][pid][invalid]")
{
    control_math::PidCore pid(makeParameters());
    control_math::PidCore reference(makeParameters());
    TEST_ASSERT_TRUE(pid.compute(0.4f, 0.1f, 0.02f).valid);
    TEST_ASSERT_TRUE(reference.compute(0.4f, 0.1f, 0.02f).valid);

    auto invalid = makeParameters();
    invalid.kp = NAN;
    TEST_ASSERT_FALSE(pid.trySetParameters(invalid));
    TEST_ASSERT_TRUE(pid.parametersValid());

    const auto expected = reference.compute(-0.2f, 0.3f, 0.01f);
    const auto actual = pid.compute(-0.2f, 0.3f, 0.01f);
    TEST_ASSERT_TRUE(actual.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected.output, actual.output);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f,
                             expected.integralTerm,
                             actual.integralTerm);
}

TEST_CASE("PidCore rejects raw integral overflow before clamping",
          "[control][pid][invalid]")
{
    control_math::PidParameters parameters = {};
    parameters.ki = 1.0e38f;
    parameters.outputMin = -FLT_MAX;
    parameters.outputMax = FLT_MAX;
    parameters.integralMin = -FLT_MAX;
    parameters.integralMax = FLT_MAX;

    control_math::PidCore pid(parameters);
    control_math::PidCore reference(parameters);
    for (int i = 0; i < 3; ++i) {
        TEST_ASSERT_TRUE(pid.compute(1.0f, 0.0f, 1.0f).valid);
        TEST_ASSERT_TRUE(reference.compute(1.0f, 0.0f, 1.0f).valid);
    }

    // The raw fourth integral sum exceeds FLT_MAX.  It must be rejected
    // before min/max clamping turns the overflow into a plausible value.
    const auto invalid = pid.compute(1.0f, 0.0f, 1.0f);
    TEST_ASSERT_FALSE(invalid.valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, invalid.output);

    const auto expected = reference.compute(0.0f, 0.0f, 0.1f);
    const auto actual = pid.compute(0.0f, 0.0f, 0.1f);
    TEST_ASSERT_TRUE(expected.valid);
    TEST_ASSERT_TRUE(actual.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected.output, actual.output);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f,
                             expected.integralTerm,
                             actual.integralTerm);
}

TEST_CASE("normalized effort uses the measured PWM deadzone at the actuator boundary", "[control][actuator]")
{
    constexpr uint32_t maxDuty = 1023;
    constexpr uint32_t deadzoneDuty = 500;

    TEST_ASSERT_EQUAL_UINT32(0,
        control_math::effortToPwmDuty(0.0f, maxDuty, deadzoneDuty));
    TEST_ASSERT_EQUAL_UINT32(deadzoneDuty,
        control_math::effortToPwmDuty(0.0011f, maxDuty, deadzoneDuty));
    TEST_ASSERT_EQUAL_UINT32(deadzoneDuty,
        control_math::effortToPwmDuty(-0.0011f, maxDuty, deadzoneDuty));
    TEST_ASSERT_EQUAL_UINT32(761,
        control_math::effortToPwmDuty(0.5f, maxDuty, deadzoneDuty));
    TEST_ASSERT_EQUAL_UINT32(maxDuty,
        control_math::effortToPwmDuty(2.0f, maxDuty, deadzoneDuty));

    // The strategy output remains normalized effort. MotorService applies
    // this mapping exactly once for both NestedPid and longitudinal control.
}
