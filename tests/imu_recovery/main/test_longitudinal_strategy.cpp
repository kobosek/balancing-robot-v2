#include "unity.h"
#include "LongitudinalCascadeBalanceStrategy.hpp"
#include <cmath>

namespace {
ConfigData makeConfig()
{
    ConfigData config;
    config.control.strategies.longitudinal_cascade.pitch = {
        0.2f, 0.0f, 0.01f, -1.0f, 1.0f, -1.0f, 1.0f
    };
    config.control.strategies.longitudinal_cascade.velocity = {
        0.1f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0f, 1.0f
    };
    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.pitch_trim_deg = 0.5f;
    longitudinal.max_pitch_offset_deg = 2.0f;
    longitudinal.max_pitch_rate_dps = 10.0f;
    longitudinal.max_effort = 0.6f;
    longitudinal.sync_max_effort = 0.2f;
    longitudinal.configured = true;
    return config;
}
}

TEST_CASE("longitudinal pitch baseline applies trim, pitch limits and effort limit", "[control][longitudinal]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    strategy.applyConfig(makeConfig());

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.currentPitch_deg = -10.0f;
    input.targetPitchOffset_deg = 10.0f;
    const MotorEffort effort = strategy.update(input);

    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.6f, effort.left);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, effort.left, effort.right);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.5f, strategy.getLastTargetPitchDeg());
    TEST_ASSERT_FALSE(strategy.isYawControlEnabled());
    const auto diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_TRUE(diagnostics.valid);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceStrategyId::LONGITUDINAL_CASCADE),
                          static_cast<int>(diagnostics.strategyId));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::PITCH_BASELINE),
                          static_cast<int>(diagnostics.phase));
    TEST_ASSERT_TRUE(diagnostics.targetPitchClamped);
    TEST_ASSERT_TRUE(diagnostics.targetPitchRateLimited);
    TEST_ASSERT_TRUE(diagnostics.balanceSaturated);
    TEST_ASSERT_FALSE(diagnostics.positionLoopEnabled);
    TEST_ASSERT_FALSE(diagnostics.velocityLoopEnabled);
}

TEST_CASE("longitudinal target pitch is slew limited from trim", "[control][longitudinal]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    strategy.applyConfig(makeConfig());

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.targetPitchOffset_deg = 2.0f;
    strategy.update(input);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.5f, strategy.getLastTargetPitchDeg());

    strategy.update(input);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 2.5f, strategy.getLastTargetPitchDeg());
}

TEST_CASE("longitudinal mixer preserves balance mean and reserves headroom for sync", "[control][longitudinal]")
{
    const auto mixed = LongitudinalCascadeBalanceStrategy::mixEfforts(0.9f, 0.5f, 1.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.1f, mixed.syncEffort);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.8f, mixed.left);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.0f, mixed.right);
    TEST_ASSERT_TRUE(mixed.syncLimited);
}

TEST_CASE("longitudinal baseline rejects invalid input and unconfigured state", "[control][longitudinal]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    BalanceControlInput input = {};
    input.dt = 0.01f;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, strategy.update(input).left);

    strategy.applyConfig(makeConfig());
    input.currentPitch_deg = NAN;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, strategy.update(input).left);
}
