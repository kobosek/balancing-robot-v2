#include "unity.h"
#include "NestedPidBalanceStrategy.hpp"
#include <algorithm>
#include <cfloat>
#include <cmath>

namespace {

struct ReferencePid {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float outputMin = -1.0f;
    float outputMax = 1.0f;
    float integralMin = -1.0f;
    float integralMax = 1.0f;
    float integral = 0.0f;
    float lastError = 0.0f;

    void reset()
    {
        integral = 0.0f;
        lastError = 0.0f;
    }

    float compute(float setpoint, float measurement, float dt)
    {
        const float error = setpoint - measurement;
        integral += (error + lastError) * 0.5f * dt * ki;
        integral = std::max(integralMin, std::min(integral, integralMax));
        const float derivative = dt > 1e-6f
            ? kd * (error - lastError) / dt : 0.0f;
        lastError = error;
        return std::max(outputMin,
                        std::min(outputMax, kp * error + integral + derivative));
    }

    float computeWithMeasurementRate(float setpoint,
                                     float measurement,
                                     float measurementRate,
                                     float dt)
    {
        const float error = setpoint - measurement;
        integral += (error + lastError) * 0.5f * dt * ki;
        integral = std::max(integralMin, std::min(integral, integralMax));
        lastError = error;
        return std::max(outputMin,
                        std::min(outputMax,
                                 kp * error + integral - kd * measurementRate));
    }
};

ReferencePid makeReference(const PIDConfig& config)
{
    return {
        config.pid_kp,
        config.pid_ki,
        config.pid_kd,
        config.pid_output_min,
        config.pid_output_max,
        config.pid_iterm_min,
        config.pid_iterm_max
    };
}

struct ReferenceOutput {
    float left = 0.0f;
    float right = 0.0f;
    float leftSetpoint = 0.0f;
    float rightSetpoint = 0.0f;
    double targetYaw = 0.0;
    float desiredYawRate = 0.0f;
};

struct ReferenceNestedPid {
    ReferencePid angle;
    ReferencePid speedLeft;
    ReferencePid speedRight;
    ReferencePid yawAngle;
    ReferencePid yawRate;
    bool yawEnabled = false;
    bool hasTargetYaw = false;
    double targetYaw = 0.0;
    float angleOutputMin = 0.0f;
    float angleOutputMax = 0.0f;
    float wheelRadiusM = 0.0f;
    float wheelbaseM = 0.0f;

    void reset()
    {
        angle.reset();
        speedLeft.reset();
        speedRight.reset();
        yawAngle.reset();
        yawRate.reset();
        yawEnabled = false;
        hasTargetYaw = false;
        targetYaw = 0.0;
    }

    ReferenceOutput update(const BalanceControlInput& input)
    {
        ReferenceOutput output;
        const float baseSpeed = angle.computeWithMeasurementRate(
            input.targetPitchOffset_deg,
            input.currentPitch_deg,
            input.currentPitchRate_dps,
            input.dt);

        if (!yawEnabled) {
            hasTargetYaw = false;
            targetYaw = input.currentYaw_deg;
        } else if (!hasTargetYaw) {
            targetYaw = input.currentYaw_deg;
            hasTargetYaw = true;
            yawAngle.reset();
            yawRate.reset();
        }

        if (yawEnabled && std::fabs(input.targetAngularVelocity_dps) > 1e-3f) {
            targetYaw += static_cast<double>(input.targetAngularVelocity_dps) * input.dt;
        }

        const float targetYawFloat = yawEnabled
            ? static_cast<float>(targetYaw) : input.currentYaw_deg;
        const float yawCorrectionRate = yawEnabled
            ? yawAngle.computeWithMeasurementRate(targetYawFloat,
                                                  input.currentYaw_deg,
                                                  input.currentYawRate_dps,
                                                  input.dt)
            : 0.0f;
        const float desiredYawRate = yawEnabled
            ? input.targetAngularVelocity_dps + yawCorrectionRate
            : input.targetAngularVelocity_dps;
        const float turnDiff = wheelRadiusM > 1e-5f
            ? (wheelbaseM / (2.0f * wheelRadiusM)) * desiredYawRate : 0.0f;
        const float yawCorrection = yawEnabled
            ? yawRate.compute(desiredYawRate,
                              input.currentYawRate_dps,
                              input.dt)
            : 0.0f;

        output.leftSetpoint = std::max(angleOutputMin,
            std::min(angleOutputMax, baseSpeed - turnDiff - yawCorrection));
        output.rightSetpoint = std::max(angleOutputMin,
            std::min(angleOutputMax, baseSpeed + turnDiff + yawCorrection));
        output.left = std::max(-1.0f, std::min(1.0f,
            speedLeft.compute(output.leftSetpoint,
                              input.currentSpeedLeft_dps,
                              input.dt)));
        output.right = std::max(-1.0f, std::min(1.0f,
            speedRight.compute(output.rightSetpoint,
                               input.currentSpeedRight_dps,
                               input.dt)));
        output.targetYaw = yawEnabled ? targetYaw : input.currentYaw_deg;
        output.desiredYawRate = desiredYawRate;
        return output;
    }
};

ConfigData makeNestedConfig(bool yawEnabled)
{
    ConfigData config;
    auto& nested = config.control.strategies.nested_pid;
    nested.angle = {1.2f, 0.35f, 0.08f, -120.0f, 120.0f, -30.0f, 30.0f};
    nested.speed_left = {0.02f, 0.10f, 0.003f, -0.9f, 0.9f, -0.5f, 0.5f};
    nested.speed_right = {0.025f, 0.08f, 0.002f, -0.9f, 0.9f, -0.5f, 0.5f};
    nested.yaw_angle = {1.4f, 0.0f, 0.04f, -50.0f, 50.0f, -20.0f, 20.0f};
    nested.yaw_rate = {0.03f, 0.12f, 0.001f, -30.0f, 30.0f, -10.0f, 10.0f};
    nested.yaw_control_enabled = yawEnabled;
    nested.max_target_pitch_offset_deg = 5.0f;
    config.control.max_target_pitch_offset_deg = nested.max_target_pitch_offset_deg;
    config.control.yaw_control_enabled = yawEnabled;
    config.encoder.wheel_diameter_mm = 64.0f;
    config.dimensions.wheelbase_m = 0.18f;
    return config;
}

void assertOutputMatches(const ReferenceOutput& expected,
                         const NestedPidBalanceStrategy& actual)
{
    const auto diagnostics = actual.getDiagnostics();
    TEST_ASSERT_TRUE(diagnostics.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, expected.left, diagnostics.leftEffort);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, expected.right, diagnostics.rightEffort);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, expected.leftSetpoint,
                             actual.getLastSpeedSetpointLeftDPS());
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, expected.rightSetpoint,
                             actual.getLastSpeedSetpointRightDPS());
    TEST_ASSERT_FLOAT_WITHIN(0.00001f,
                             static_cast<float>(expected.targetYaw),
                             actual.getLastTargetYawDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, expected.desiredYawRate,
                             actual.getLastDesiredYawRateDPS());
}

} // namespace

TEST_CASE("NestedPid matches the preserved reference with yaw disabled",
          "[control][nested_pid][reference]")
{
    const ConfigData config = makeNestedConfig(false);
    NestedPidBalanceStrategy strategy;
    strategy.applyConfig(config);

    ReferenceNestedPid reference;
    const auto& nested = config.control.strategies.nested_pid;
    reference.angle = makeReference(nested.angle);
    reference.speedLeft = makeReference(nested.speed_left);
    reference.speedRight = makeReference(nested.speed_right);
    reference.yawAngle = makeReference(nested.yaw_angle);
    reference.yawRate = makeReference(nested.yaw_rate);
    reference.angleOutputMin = nested.angle.pid_output_min;
    reference.angleOutputMax = nested.angle.pid_output_max;
    reference.wheelRadiusM = config.encoder.wheel_diameter_mm / 2000.0f;
    reference.wheelbaseM = config.dimensions.wheelbase_m;

    const BalanceControlInput inputs[] = {
        {.dt = 0.005f, .currentPitch_deg = -1.0f,
         .currentPitchRate_dps = 4.0f, .currentSpeedLeft_dps = 10.0f,
         .currentSpeedRight_dps = -8.0f, .targetPitchOffset_deg = 2.0f},
        {.dt = 0.010f, .currentPitch_deg = 0.6f,
         .currentPitchRate_dps = -3.0f, .currentSpeedLeft_dps = 12.0f,
         .currentSpeedRight_dps = 9.0f, .targetPitchOffset_deg = -1.0f},
        {.dt = 0.007f, .currentPitch_deg = -0.3f,
         .currentPitchRate_dps = 1.5f, .currentSpeedLeft_dps = -4.0f,
         .currentSpeedRight_dps = 6.0f, .targetPitchOffset_deg = 0.25f}
    };

    for (const auto& input : inputs) {
        const auto expected = reference.update(input);
        strategy.update(input);
        assertOutputMatches(expected, strategy);
    }
}

TEST_CASE("NestedPid matches the preserved reference through yaw heading capture and turn",
          "[control][nested_pid][yaw][reference]")
{
    const ConfigData config = makeNestedConfig(true);
    NestedPidBalanceStrategy strategy;
    strategy.applyConfig(config);

    ReferenceNestedPid reference;
    const auto& nested = config.control.strategies.nested_pid;
    reference.angle = makeReference(nested.angle);
    reference.speedLeft = makeReference(nested.speed_left);
    reference.speedRight = makeReference(nested.speed_right);
    reference.yawAngle = makeReference(nested.yaw_angle);
    reference.yawRate = makeReference(nested.yaw_rate);
    reference.yawEnabled = true;
    reference.angleOutputMin = nested.angle.pid_output_min;
    reference.angleOutputMax = nested.angle.pid_output_max;
    reference.wheelRadiusM = config.encoder.wheel_diameter_mm / 2000.0f;
    reference.wheelbaseM = config.dimensions.wheelbase_m;

    const BalanceControlInput inputs[] = {
        {.dt = 0.005f, .currentPitch_deg = 0.0f,
         .currentPitchRate_dps = 0.0f, .currentYaw_deg = 10.0f,
         .currentYawRate_dps = 0.0f, .currentSpeedLeft_dps = 0.0f,
         .currentSpeedRight_dps = 0.0f, .targetAngularVelocity_dps = 0.0f},
        {.dt = 0.010f, .currentPitch_deg = 0.4f,
         .currentPitchRate_dps = -2.0f, .currentYaw_deg = 9.0f,
         .currentYawRate_dps = 5.0f, .currentSpeedLeft_dps = 6.0f,
         .currentSpeedRight_dps = 12.0f, .targetPitchOffset_deg = 0.5f,
         .targetAngularVelocity_dps = 18.0f},
        {.dt = 0.007f, .currentPitch_deg = -0.2f,
         .currentPitchRate_dps = 1.0f, .currentYaw_deg = 9.6f,
         .currentYawRate_dps = -3.0f, .currentSpeedLeft_dps = -5.0f,
         .currentSpeedRight_dps = 4.0f, .targetPitchOffset_deg = -0.25f,
         .targetAngularVelocity_dps = -12.0f},
        {.dt = 0.005f, .currentPitch_deg = 0.1f,
         .currentPitchRate_dps = 0.5f, .currentYaw_deg = 9.3f,
         .currentYawRate_dps = 2.0f, .currentSpeedLeft_dps = 2.0f,
         .currentSpeedRight_dps = 1.0f, .targetAngularVelocity_dps = 0.0f}
    };

    for (const auto& input : inputs) {
        const auto expected = reference.update(input);
        strategy.update(input);
        assertOutputMatches(expected, strategy);
    }

    strategy.reset();
    reference.reset();
    reference.yawEnabled = true;
    const auto expectedAfterReset = reference.update(inputs[1]);
    strategy.update(inputs[1]);
    assertOutputMatches(expectedAfterReset, strategy);
}

TEST_CASE("NestedPid rejects a nonfinite step without contaminating controller history",
          "[control][nested_pid][reference][invalid]")
{
    const ConfigData config = makeNestedConfig(false);
    NestedPidBalanceStrategy strategy;
    strategy.applyConfig(config);

    ReferenceNestedPid reference;
    const auto& nested = config.control.strategies.nested_pid;
    reference.angle = makeReference(nested.angle);
    reference.speedLeft = makeReference(nested.speed_left);
    reference.speedRight = makeReference(nested.speed_right);
    reference.yawAngle = makeReference(nested.yaw_angle);
    reference.yawRate = makeReference(nested.yaw_rate);
    reference.angleOutputMin = nested.angle.pid_output_min;
    reference.angleOutputMax = nested.angle.pid_output_max;
    reference.wheelRadiusM = config.encoder.wheel_diameter_mm / 2000.0f;
    reference.wheelbaseM = config.dimensions.wheelbase_m;

    BalanceControlInput first = {
        .dt = 0.005f,
        .currentPitch_deg = -0.6f,
        .currentPitchRate_dps = 2.0f,
        .currentSpeedLeft_dps = 4.0f,
        .currentSpeedRight_dps = -3.0f,
        .targetPitchOffset_deg = 1.0f
    };
    const auto expectedFirst = reference.update(first);
    strategy.update(first);
    assertOutputMatches(expectedFirst, strategy);

    BalanceControlInput invalid = first;
    invalid.currentPitch_deg = NAN;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, strategy.update(invalid).left);
    TEST_ASSERT_FALSE(strategy.getDiagnostics().valid);

    BalanceControlInput next = first;
    next.dt = 0.01f;
    next.currentPitch_deg = 0.2f;
    next.currentPitchRate_dps = -1.0f;
    next.currentSpeedLeft_dps = 6.0f;
    next.currentSpeedRight_dps = 5.0f;
    next.targetPitchOffset_deg = -0.25f;
    const auto expectedNext = reference.update(next);
    strategy.update(next);
    assertOutputMatches(expectedNext, strategy);
}

TEST_CASE("NestedPid rejects a downstream overflow without partial angle or yaw state",
          "[control][nested_pid][reference][invalid][transaction]")
{
    const ConfigData config = makeNestedConfig(false);
    ConfigData highGainConfig = config;
    highGainConfig.control.strategies.nested_pid.speed_left.pid_kp = 500.0f;

    NestedPidBalanceStrategy strategy;
    strategy.applyConfig(highGainConfig);

    ReferenceNestedPid reference;
    const auto& nested = highGainConfig.control.strategies.nested_pid;
    reference.angle = makeReference(nested.angle);
    reference.speedLeft = makeReference(nested.speed_left);
    reference.speedRight = makeReference(nested.speed_right);
    reference.yawAngle = makeReference(nested.yaw_angle);
    reference.yawRate = makeReference(nested.yaw_rate);
    reference.angleOutputMin = nested.angle.pid_output_min;
    reference.angleOutputMax = nested.angle.pid_output_max;
    reference.wheelRadiusM = highGainConfig.encoder.wheel_diameter_mm / 2000.0f;
    reference.wheelbaseM = highGainConfig.dimensions.wheelbase_m;

    BalanceControlInput first = {
        .dt = 0.005f,
        .currentPitch_deg = -0.4f,
        .currentPitchRate_dps = 1.0f,
        .currentSpeedLeft_dps = 8.0f,
        .currentSpeedRight_dps = 3.0f,
        .targetPitchOffset_deg = 0.8f
    };
    const auto expectedFirst = reference.update(first);
    strategy.update(first);
    assertOutputMatches(expectedFirst, strategy);

    // The sample is finite, but the intentionally large configured gain
    // overflows only in the left wheel branch.  The complete strategy step
    // must be rejected before the angle controller commits its new state.
    BalanceControlInput overflowing = first;
    overflowing.currentPitch_deg = 0.2f;
    overflowing.currentSpeedLeft_dps = -FLT_MAX;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, strategy.update(overflowing).left);
    TEST_ASSERT_FALSE(strategy.getDiagnostics().valid);

    BalanceControlInput next = first;
    next.dt = 0.01f;
    next.currentPitch_deg = 0.1f;
    next.currentPitchRate_dps = -0.5f;
    next.currentSpeedLeft_dps = 6.0f;
    next.currentSpeedRight_dps = 4.0f;
    next.targetPitchOffset_deg = -0.2f;
    const auto expectedNext = reference.update(next);
    strategy.update(next);
    assertOutputMatches(expectedNext, strategy);
}
