#include "unity.h"
#include "LongitudinalCascadeBalanceStrategy.hpp"
#include "LongitudinalMotionProfile.hpp"
#include <cfloat>
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

ConfigData makeMotionConfig()
{
    ConfigData config = makeConfig();
    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.velocity = {
        1.0f, 1.0f, 0.0f, -2.0f, 2.0f, -2.0f, 2.0f
    };
    longitudinal.max_velocity_mps = 1.0f;
    longitudinal.max_acceleration_mps2 = 2.0f;
    longitudinal.max_deceleration_mps2 = 1.0f;
    longitudinal.max_pitch_offset_deg = 2.0f;
    longitudinal.max_pitch_rate_dps = 100.0f;
    longitudinal.loop_mode = LongitudinalLoopMode::VELOCITY;
    return config;
}

ConfigData makePositionHoldConfig()
{
    ConfigData config = makeMotionConfig();
    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.loop_mode = LongitudinalLoopMode::POSITION_HOLD;
    longitudinal.position_kp = 2.0f;
    longitudinal.max_hold_velocity_mps = 0.2f;
    longitudinal.hold_position_deadband_m = 0.01f;
    longitudinal.hold_settle_time_ms = 100;
    return config;
}

LongitudinalOdometryResult validOdometry(float positionM = 0.0f,
                                         float velocityMps = 0.0f)
{
    LongitudinalOdometryResult odometry = {};
    odometry.status = LongitudinalOdometryUpdateStatus::ACCEPTED;
    odometry.positionValid = true;
    odometry.velocityValid = true;
    odometry.odometryValid = true;
    odometry.leftFeedbackValid = true;
    odometry.rightFeedbackValid = true;
    odometry.positionM = positionM;
    odometry.leftVelocityMps = velocityMps;
    odometry.rightVelocityMps = velocityMps;
    odometry.velocityMps = velocityMps;
    return odometry;
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

TEST_CASE("longitudinal motion profile ramps velocity and enables PI loop", "[control][longitudinal]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    strategy.applyConfig(makeMotionConfig());

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.odometry = validOdometry();
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = false;
    input.motion.targetVelocityMps = 0.5f;

    const MotorEffort effort = strategy.update(input);
    const auto diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_TRUE(std::isfinite(effort.left));
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.2f, diagnostics.targetVelocityMps);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::DRIVE),
                          static_cast<int>(diagnostics.phase));
    TEST_ASSERT_TRUE(diagnostics.velocityLoopEnabled);
    TEST_ASSERT_TRUE(diagnostics.motionCommandFresh);
}

TEST_CASE("longitudinal timeout brakes and captures the stopped position", "[control][longitudinal]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    strategy.applyConfig(makeMotionConfig());

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.odometry = validOdometry(0.0f, 0.0f);
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = false;
    input.motion.targetVelocityMps = 0.5f;
    strategy.update(input);
    strategy.update(input);

    input.motion.fresh = false;
    input.odometry.positionM = 1.25f;
    auto diagnostics = strategy.getDiagnostics();
    (void)diagnostics;
    strategy.update(input);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::BRAKE),
                          static_cast<int>(strategy.getDiagnostics().phase));
    strategy.update(input);
    strategy.update(input);
    strategy.update(input);
    strategy.update(input);
    strategy.update(input);
    strategy.update(input);
    strategy.update(input);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::HOLD),
                          static_cast<int>(strategy.getDiagnostics().phase));
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.25f,
                             static_cast<float>(strategy.getLastHoldPositionM()));
}

TEST_CASE("longitudinal velocity mode never falls back to pitch baseline without a command", "[control][longitudinal]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    strategy.applyConfig(makeMotionConfig());

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.currentPitch_deg = -5.0f;
    input.targetPitchOffset_deg = 2.0f;
    input.odometry = validOdometry();

    const MotorEffort effort = strategy.update(input);
    const auto diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_TRUE(std::isfinite(effort.left));
    TEST_ASSERT_TRUE(std::isfinite(effort.right));
    TEST_ASSERT_FALSE(diagnostics.motionCommandValid);
    TEST_ASSERT_TRUE(diagnostics.velocityLoopEnabled);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, diagnostics.targetVelocityMps);
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(BalanceControlPhase::PITCH_BASELINE),
                          static_cast<int>(diagnostics.phase));
}

TEST_CASE("longitudinal HOLD waits for both wheels to stop", "[control][longitudinal]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    ConfigData config = makeMotionConfig();
    config.control.strategies.longitudinal_cascade.hold_settle_time_ms = 100;
    strategy.applyConfig(config);

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.odometry = validOdometry(0.0f, 0.0f);
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = false;
    input.motion.targetVelocityMps = 0.2f;
    strategy.update(input);

    input.motion.stop = true;
    input.motion.targetVelocityMps = 0.0f;
    input.odometry.leftVelocityMps = 0.0f;
    input.odometry.rightVelocityMps = 0.2f;
    strategy.update(input);
    strategy.update(input);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::BRAKE),
                          static_cast<int>(strategy.getDiagnostics().phase));

    input.odometry.rightVelocityMps = 0.0f;
    strategy.update(input);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::BRAKE),
                          static_cast<int>(strategy.getDiagnostics().phase));
    strategy.update(input);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::HOLD),
                          static_cast<int>(strategy.getDiagnostics().phase));
}

TEST_CASE("longitudinal command arm mismatch can only brake and never drive", "[control][longitudinal]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    strategy.applyConfig(makeMotionConfig());

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.controlArmId = 7;
    input.odometry = validOdometry();
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = false;
    input.motion.targetVelocityMps = 0.5f;
    input.motion.armId = 6;

    strategy.update(input);
    const auto diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_FALSE(diagnostics.motionCommandValid);
    TEST_ASSERT_FALSE(diagnostics.motionCommandFresh);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::BRAKE),
                          static_cast<int>(diagnostics.phase));
}

TEST_CASE("longitudinal request limiter reduces speed near pitch limit and releases with hysteresis", "[control][longitudinal]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    ConfigData config = makeMotionConfig();
    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.motion_request_limit_enabled = true;
    longitudinal.motion_request_limit_pitch_start_deg = 0.4f;
    longitudinal.motion_request_limit_pitch_full_deg = 1.0f;
    longitudinal.motion_request_limit_pitch_release_deg = 0.2f;
    longitudinal.motion_request_limit_effort_start = 0.9f;
    longitudinal.motion_request_limit_effort_full = 1.0f;
    longitudinal.motion_request_limit_effort_release = 0.8f;
    longitudinal.motion_request_limit_min_scale = 0.2f;
    strategy.applyConfig(config);

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.controlArmId = 1;
    input.odometry = validOdometry();
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = false;
    input.motion.targetVelocityMps = 0.5f;
    input.motion.armId = 1;

    input.currentPitch_deg = longitudinal.pitch_trim_deg;
    strategy.update(input);
    TEST_ASSERT_FALSE(strategy.getDiagnostics().motionRequestLimited);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.2f,
                             strategy.getLastTargetVelocityMps());

    input.currentPitch_deg = longitudinal.pitch_trim_deg + 0.7f;
    strategy.update(input);
    const auto limited = strategy.getDiagnostics();
    TEST_ASSERT_TRUE(limited.motionRequestLimited);
    TEST_ASSERT_TRUE(limited.velocityTargetClamped);
    TEST_ASSERT_TRUE(strategy.getLastTargetVelocityMps() < 0.4f);

    // Crossing only the release pitch threshold is insufficient while the
    // limiter is latched; both stability signals must be back in their safe
    // bands before the full request is restored.
    input.currentPitch_deg = longitudinal.pitch_trim_deg + 0.3f;
    strategy.update(input);
    TEST_ASSERT_TRUE(strategy.getDiagnostics().motionRequestLimited);

    input.currentPitch_deg = longitudinal.pitch_trim_deg + 0.1f;
    strategy.update(input);
    TEST_ASSERT_FALSE(strategy.getDiagnostics().motionRequestLimited);
}

TEST_CASE("position hold captures once and returns through the velocity profile", "[control][longitudinal][position]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    strategy.applyConfig(makePositionHoldConfig());

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.odometry = validOdometry();
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = true;
    input.motion.targetVelocityMps = 0.0f;

    // The first stable frame starts the observation window.  Its preceding
    // dt is not counted toward hold_settle_time_ms.
    strategy.update(input);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::BRAKE),
                          static_cast<int>(strategy.getDiagnostics().phase));
    strategy.update(input);
    auto diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::HOLD),
                          static_cast<int>(diagnostics.phase));
    TEST_ASSERT_TRUE(diagnostics.positionTargetValid);
    TEST_ASSERT_FALSE(diagnostics.positionHoldActive);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f,
                             static_cast<float>(strategy.getLastHoldPositionM()));

    // A settled displacement arms the P loop.  The return command is limited
    // by max_hold_velocity_mps and remains inside HOLD while the wheels move.
    input.odometry.positionM = 0.3f;
    strategy.update(input);
    diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_TRUE(diagnostics.positionHoldActive);

    // Once armed, the loop remains in HOLD even while its profiled return
    // command produces a non-zero wheel velocity.
    input.odometry.leftVelocityMps = 0.1f;
    input.odometry.rightVelocityMps = 0.1f;
    strategy.update(input);
    diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::HOLD),
                          static_cast<int>(diagnostics.phase));
    TEST_ASSERT_TRUE(diagnostics.positionHoldActive);
    TEST_ASSERT_TRUE(diagnostics.positionLoopEnabled);
    TEST_ASSERT_TRUE(diagnostics.holdVelocityRequestMps < 0.0f);
    TEST_ASSERT_TRUE(diagnostics.holdVelocityTargetMps < 0.0f);
    TEST_ASSERT_TRUE(std::fabs(diagnostics.holdVelocityTargetMps) <= 0.2f);

    // Returning inside the release band drops the correction without moving
    // the captured target or integrating a stale route.
    input.odometry.positionM = 0.005;
    input.odometry.leftVelocityMps = 0.0f;
    input.odometry.rightVelocityMps = 0.0f;
    strategy.update(input);
    diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_FALSE(diagnostics.positionHoldActive);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f,
                             diagnostics.holdVelocityRequestMps);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f,
                             static_cast<float>(strategy.getLastHoldPositionM()));

    // A new external DRIVE command disables the position correction.
    input.motion.stop = false;
    input.motion.targetVelocityMps = 0.2f;
    strategy.update(input);
    diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::DRIVE),
                          static_cast<int>(diagnostics.phase));
    TEST_ASSERT_FALSE(diagnostics.positionLoopEnabled);
    TEST_ASSERT_FALSE(diagnostics.positionHoldActive);
}

TEST_CASE("wheel synchronization keeps one distance reference through braking and deadbands noise", "[control][longitudinal][sync]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    ConfigData config = makeMotionConfig();
    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.sync_enabled = true;
    longitudinal.sync_kp = 0.5f;
    longitudinal.sync_kd = 0.2f;
    longitudinal.sync_position_deadband_m = 0.01f;
    longitudinal.sync_velocity_deadband_mps = 0.01f;
    longitudinal.sync_max_effort = 0.2f;
    strategy.applyConfig(config);

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.odometry = validOdometry();
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = false;
    input.motion.targetVelocityMps = 0.2f;
    strategy.update(input);
    auto diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_TRUE(diagnostics.synchronizationEnabled);
    TEST_ASSERT_TRUE(diagnostics.synchronizationTargetValid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f,
                             diagnostics.requestedSyncEffort);

    // Both errors are inside their independent noise bands.
    input.odometry.distanceDifferenceM = 0.005;
    input.odometry.velocityDifferenceMps = 0.005f;
    strategy.update(input);
    diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f,
                             diagnostics.requestedSyncEffort);

    // A positive left-minus-right error slows the left wheel and speeds the
    // right wheel.  The original dTarget remains zero.
    input.odometry.distanceDifferenceM = 0.1;
    input.odometry.velocityDifferenceMps = 0.05f;
    strategy.update(input);
    diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_TRUE(diagnostics.synchronizationEnabled);
    TEST_ASSERT_TRUE(diagnostics.requestedSyncEffort > 0.0f);
    TEST_ASSERT_TRUE(diagnostics.leftEffort < diagnostics.rightEffort);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f,
                             static_cast<float>(diagnostics.distanceDifferenceTargetM));

    // Stopping preserves the same differential reference instead of
    // recapturing the already accumulated asymmetry.
    input.motion.stop = true;
    input.motion.targetVelocityMps = 0.0f;
    input.odometry.leftVelocityMps = 0.0f;
    input.odometry.rightVelocityMps = 0.0f;
    strategy.update(input);
    diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::BRAKE),
                          static_cast<int>(diagnostics.phase));
    TEST_ASSERT_TRUE(diagnostics.synchronizationTargetValid);
    TEST_ASSERT_TRUE(diagnostics.requestedSyncEffort > 0.0f);

    // A later DRIVE segment captures a new reference exactly once.
    input.motion.stop = false;
    input.motion.targetVelocityMps = 0.2f;
    input.odometry.distanceDifferenceM = 0.2;
    input.odometry.velocityDifferenceMps = 0.0f;
    strategy.update(input);
    diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.2f,
                             static_cast<float>(diagnostics.distanceDifferenceTargetM));
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f,
                             diagnostics.requestedSyncEffort);
}

TEST_CASE("position hold deadband uses Schmitt hysteresis without chatter",
          "[control][longitudinal][position][hysteresis]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    auto config = makePositionHoldConfig();
    config.control.strategies.longitudinal_cascade.hold_position_deadband_m = 0.01f;
    strategy.applyConfig(config);

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.odometry = validOdometry();
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = true;
    strategy.update(input);
    strategy.update(input);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::HOLD),
                          static_cast<int>(strategy.getDiagnostics().phase));

    input.odometry.positionM = 0.015f;
    strategy.update(input);
    TEST_ASSERT_FALSE(strategy.getDiagnostics().positionHoldActive);

    input.odometry.positionM = 0.03f;
    strategy.update(input);
    TEST_ASSERT_TRUE(strategy.getDiagnostics().positionHoldActive);

    // Between release and entry thresholds, the active correction remains on.
    input.odometry.positionM = 0.015f;
    strategy.update(input);
    TEST_ASSERT_TRUE(strategy.getDiagnostics().positionHoldActive);

    input.odometry.positionM = 0.005f;
    strategy.update(input);
    TEST_ASSERT_FALSE(strategy.getDiagnostics().positionHoldActive);
}

TEST_CASE("longitudinal continuity generation cannot reuse an old hold target", "[control][longitudinal][continuity]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    strategy.applyConfig(makePositionHoldConfig());

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.odometry = validOdometry();
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = true;
    strategy.update(input);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::BRAKE),
                          static_cast<int>(strategy.getDiagnostics().phase));
    strategy.update(input);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::HOLD),
                          static_cast<int>(strategy.getDiagnostics().phase));

    input.odometry.positionM = 0.3;
    strategy.update(input);
    TEST_ASSERT_TRUE(strategy.getDiagnostics().positionHoldActive);

    // A valid sample from a new continuity generation is not allowed to
    // drive toward the old sTarget.  The strategy fails closed and requires a
    // fresh session before it captures another target.
    input.odometry.generation = 1;
    input.odometry.positionM = 0.0;
    const auto effort = strategy.update(input);
    const auto fault = strategy.getDiagnostics();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, effort.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, effort.right);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::FAULT),
                          static_cast<int>(fault.phase));
    TEST_ASSERT_FALSE(fault.positionTargetValid);

    strategy.update(input);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::BRAKE),
                          static_cast<int>(strategy.getDiagnostics().phase));
    strategy.update(input);
    const auto recaptured = strategy.getDiagnostics();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceControlPhase::HOLD),
                          static_cast<int>(recaptured.phase));
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f,
                             static_cast<float>(strategy.getLastHoldPositionM()));
}

TEST_CASE("wheel synchronization recaptures direction after a profiled reversal", "[control][longitudinal][sync]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    ConfigData config = makeMotionConfig();
    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.sync_enabled = true;
    longitudinal.sync_kp = 0.5f;
    longitudinal.sync_kd = 0.2f;
    longitudinal.sync_max_effort = 0.2f;
    strategy.applyConfig(config);

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.odometry = validOdometry();
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = false;
    input.motion.targetVelocityMps = 0.2f;
    strategy.update(input);

    input.odometry.distanceDifferenceM = 0.02;
    input.motion.targetVelocityMps = -0.2f;
    strategy.update(input); // decelerates the positive profile toward zero
    strategy.update(input); // reaches zero and establishes the reverse base
    const auto diagnostics = strategy.getDiagnostics();
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.02f,
                             static_cast<float>(diagnostics.distanceDifferenceTargetM));
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f,
                             diagnostics.requestedSyncEffort);
}

TEST_CASE("longitudinal reversal passes through zero and anti-windup blocks pitch limit", "[control][longitudinal]")
{
    LongitudinalCascadeBalanceStrategy strategy;
    ConfigData config = makeMotionConfig();
    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.velocity = {0.0f, 10.0f, 0.0f, -5.0f, 5.0f, -5.0f, 5.0f};
    longitudinal.max_pitch_offset_deg = 0.1f;
    strategy.applyConfig(config);

    BalanceControlInput input = {};
    input.dt = 0.1f;
    input.odometry = validOdometry(0.0f, 0.0f);
    input.motion.valid = true;
    input.motion.fresh = true;
    input.motion.stop = false;
    input.motion.targetVelocityMps = 0.5f;
    strategy.update(input);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.2f,
                             strategy.getLastTargetVelocityMps());

    input.motion.targetVelocityMps = -0.5f;
    strategy.update(input);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.1f,
                             strategy.getLastTargetVelocityMps());
    TEST_ASSERT_TRUE(strategy.getDiagnostics().velocityAntiWindup);
    TEST_ASSERT_TRUE(strategy.getDiagnostics().velocityOutputSaturated);
    strategy.update(input);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f,
                             strategy.getLastTargetVelocityMps());
    strategy.update(input);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, -0.2f,
                             strategy.getLastTargetVelocityMps());
    // Once the candidate integral changes sign, the previous positive
    // clamp must be allowed to unwind instead of being held by a stale error
    // direction.
    TEST_ASSERT_FALSE(strategy.getDiagnostics().velocityAntiWindup);
}

TEST_CASE("longitudinal motion profile rejects an overflowing step without changing target",
          "[control][longitudinal][invalid]")
{
    LongitudinalMotionProfile profile({1.0f, 2.0f, 1.0f});
    const auto first = profile.update(0.5f, true, 0.1f);
    TEST_ASSERT_TRUE(first.valid);
    const float previousTarget = profile.targetVelocityMps();

    const auto invalid = profile.update(0.5f, true, FLT_MAX);
    TEST_ASSERT_FALSE(invalid.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, previousTarget,
                             profile.targetVelocityMps());
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, previousTarget,
                             invalid.targetVelocityMps);
}
