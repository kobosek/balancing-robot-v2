#include "LongitudinalCascadeBalanceStrategy.hpp"

#include <algorithm>
#include <cmath>

namespace {
float clampSymmetric(float value, float limit)
{
    if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0f) {
        return 0.0f;
    }
    return std::max(-limit, std::min(limit, value));
}
}

LongitudinalCascadeBalanceStrategy::LongitudinalCascadeBalanceStrategy()
    : m_pitchPid("longitudinal.pitch"),
      m_velocityPid("longitudinal.velocity")
{
    reset();
}

MotorEffort LongitudinalCascadeBalanceStrategy::update(const BalanceControlInput& input)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_last_diagnostics = {};
    m_last_diagnostics.strategyId = BalanceStrategyId::LONGITUDINAL_CASCADE;
    m_last_diagnostics.phase = m_config.configured
        ? BalanceControlPhase::PITCH_BASELINE
        : BalanceControlPhase::INACTIVE;
    if (!m_config.configured || !std::isfinite(input.dt) || input.dt <= 0.0f ||
        !std::isfinite(input.currentPitch_deg) ||
        !std::isfinite(input.currentPitchRate_dps)) {
        return {};
    }

    // An invalid command is the compatibility path used by older callers of
    // the strategy.  CommandProcessor always supplies a valid stop command
    // for an active longitudinal session, so this path never turns a stale
    // session into a drive request.
    if (!input.motion.valid) {
        return updatePitchBaseline(input);
    }
    return updateMotion(input);
}

MotorEffort LongitudinalCascadeBalanceStrategy::updatePitchBaseline(
    const BalanceControlInput& input)
{
    if (!std::isfinite(input.targetPitchOffset_deg)) {
        return {};
    }

    if (m_motion_session_initialized ||
        m_motion_phase == BalanceControlPhase::DRIVE ||
        m_motion_phase == BalanceControlPhase::BRAKE ||
        m_motion_phase == BalanceControlPhase::HOLD) {
        resetMotionState();
    }

    const float requestedPitch = clampTargetPitch(
        m_config.pitch_trim_deg + input.targetPitchOffset_deg);
    const float targetPitch = slewTargetPitch(requestedPitch, input.dt);
    const auto pitchStep = m_pitchPid.computeWithMeasurementRateDetailed(
        targetPitch,
        input.currentPitch_deg,
        input.currentPitchRate_dps,
        input.dt);
    if (!pitchStep.valid) {
        return {};
    }
    const float balanceRequested = pitchStep.output;

    // Position, velocity and wheel synchronization are intentionally zero in
    // stage D. Their state must not accumulate in this pitch-only baseline.
    const LongitudinalMixerResult mixed = mixEfforts(
        balanceRequested,
        0.0f,
        m_config.max_effort,
        m_config.sync_max_effort);

    m_last_target_pitch_deg = targetPitch;
    m_last_speed_setpoint_left_dps = 0.0f;
    m_last_speed_setpoint_right_dps = 0.0f;
    m_last_target_yaw_deg = input.currentYaw_deg;
    m_last_desired_yaw_rate_dps = 0.0f;

    m_last_diagnostics.valid = true;
    m_last_diagnostics.targetPitchValid = true;
    m_last_diagnostics.phase = BalanceControlPhase::PITCH_BASELINE;
    m_last_diagnostics.targetPitchClamped = requestedPitch !=
        (m_config.pitch_trim_deg + input.targetPitchOffset_deg);
    m_last_diagnostics.targetPitchRateLimited = targetPitch != requestedPitch;
    m_last_diagnostics.balanceSaturated = mixed.balanceSaturated;
    m_last_diagnostics.syncLimited = mixed.syncLimited;
    m_last_diagnostics.targetPitch_deg = targetPitch;
    m_last_diagnostics.requestedBalanceEffort = balanceRequested;
    m_last_diagnostics.balanceEffort = mixed.balanceEffort;
    m_last_diagnostics.requestedSyncEffort = 0.0f;
    m_last_diagnostics.syncEffort = mixed.syncEffort;
    m_last_diagnostics.leftEffort = mixed.left;
    m_last_diagnostics.rightEffort = mixed.right;

    return {mixed.left, mixed.right};
}

MotorEffort LongitudinalCascadeBalanceStrategy::updateMotion(
    const BalanceControlInput& input)
{
    const auto failMotion = [&]() -> MotorEffort {
        // A longitudinal fault must not leave a profiled target or an
        // integral alive for a later control step. RobotController still
        // latches the fault and requires a new arm before motor output can
        // resume.
        m_velocityPid.reset();
        resetMotionState();
        m_motion_phase = BalanceControlPhase::FAULT;
        m_last_diagnostics.phase = m_motion_phase;
        m_last_diagnostics.positionLoopEnabled = false;
        m_last_diagnostics.velocityLoopEnabled = false;
        m_last_diagnostics.synchronizationEnabled = false;
        m_last_diagnostics.targetVelocityMps = 0.0f;
        m_last_diagnostics.measuredVelocityMps = 0.0f;
        m_last_diagnostics.velocityCorrection_deg = 0.0f;
        m_last_diagnostics.holdPositionM = 0.0;
        return {};
    };

    m_last_diagnostics.motionCommandValid = true;
    m_last_diagnostics.motionCommandFresh = input.motion.fresh;
    m_last_diagnostics.commandVelocityMps = input.motion.targetVelocityMps;
    m_last_diagnostics.holdPositionM = m_hold_position_m;

    if (!input.odometry.odometryValid ||
        !input.odometry.positionValid ||
        !input.odometry.velocityValid ||
        !std::isfinite(input.odometry.positionM) ||
        !std::isfinite(input.odometry.velocityMps)) {
        m_velocity_anti_windup = false;
        m_last_diagnostics.velocityFeedbackValid = false;
        return failMotion();
    }

    if (!m_motion_session_initialized) {
        m_motionProfile.reset();
        m_motion_session_initialized = true;
        m_motion_phase = BalanceControlPhase::CAPTURE;
        m_hold_position_m = input.odometry.positionM;
        m_velocity_anti_windup = false;
        m_velocityPid.reset();
    }

    bool commandFresh = input.motion.fresh;
    if (input.nowUs > 0 && input.motionTimeoutUs > 0) {
        if (input.motion.receivedTimestampUs <= 0) {
            commandFresh = false;
        } else {
            const int64_t ageUs = input.nowUs - input.motion.receivedTimestampUs;
            commandFresh = commandFresh && ageUs >= 0 && ageUs <= input.motionTimeoutUs;
        }
    }

    const float commandVelocity = input.motion.targetVelocityMps;
    const bool commandFinite = std::isfinite(commandVelocity);
    const float boundedCommand = commandFinite
        ? clampSymmetric(commandVelocity, m_config.max_velocity_mps)
        : 0.0f;
    const bool driveRequested = commandFresh && !input.motion.stop &&
        commandFinite && std::fabs(boundedCommand) > 1e-5f;

    const auto profileResult = m_motionProfile.update(
        commandVelocity, driveRequested, input.dt);
    if (!profileResult.valid) {
        return failMotion();
    }

    const BalanceControlPhase previousPhase = m_motion_phase;
    if (driveRequested) {
        m_motion_phase = BalanceControlPhase::DRIVE;
    } else if (!profileResult.atRest) {
        m_motion_phase = BalanceControlPhase::BRAKE;
    } else {
        m_motion_phase = BalanceControlPhase::HOLD;
    }

    const bool enteredHold = (previousPhase == BalanceControlPhase::DRIVE ||
                              previousPhase == BalanceControlPhase::BRAKE) &&
        m_motion_phase == BalanceControlPhase::HOLD;
    if (enteredHold) {
        // The E-stage contract captures the position at the end of braking;
        // the actual P position correction is intentionally part of F.
        m_hold_position_m = input.odometry.positionM;
        m_velocityPid.reset();
        m_velocity_anti_windup = false;
    }

    const float targetVelocity = profileResult.targetVelocityMps;
    const float velocityError = targetVelocity - input.odometry.velocityMps;
    const auto velocityPreview = m_velocityPid.preview(
        targetVelocity,
        input.odometry.velocityMps,
        input.dt);
    if (!velocityPreview.valid) {
        return failMotion();
    }

    const float previewCorrection = velocityPreview.output;
    const float previewBoundedCorrection = clampSymmetric(
        previewCorrection, m_config.max_pitch_offset_deg);
    const float previewPitch = clampTargetPitch(
        m_config.pitch_trim_deg + previewBoundedCorrection);
    const float previewOffset = previewPitch - m_config.pitch_trim_deg;
    const float currentTargetPitch = m_target_pitch_initialized
        ? m_targetPitch_deg : m_config.pitch_trim_deg;
    const float currentOffset = currentTargetPitch - m_config.pitch_trim_deg;
    const float reachableOffset = [&] {
        if (!std::isfinite(m_config.max_pitch_rate_dps) ||
            m_config.max_pitch_rate_dps <= 0.0f) {
            return previewOffset;
        }
        return currentOffset + clampSymmetric(
            previewOffset - currentOffset,
            m_config.max_pitch_rate_dps * input.dt);
    }();
    // Preview the downstream pitch and mixer limits before committing the
    // velocity integral. This keeps the anti-windup decision in the same
    // units as the velocity output (degrees of pitch), while still allowing
    // the integral to unwind when the error changes direction.
    const auto pitchPreview = m_pitchPid.previewWithMeasurementRate(
        m_config.pitch_trim_deg + reachableOffset,
        input.currentPitch_deg,
        input.currentPitchRate_dps,
        input.dt);
    if (!pitchPreview.valid) {
        return failMotion();
    }
    const auto mixerPreview = mixEfforts(
        pitchPreview.output,
        0.0f,
        m_config.max_effort,
        m_config.sync_max_effort);
    const bool previewFollowsError =
        (velocityError > 1e-5f && previewOffset > 1e-5f) ||
        (velocityError < -1e-5f && previewOffset < -1e-5f);
    const bool pitchLimitBlocks = previewFollowsError &&
        ((velocityError > 1e-5f && previewOffset > reachableOffset + 1e-5f) ||
         (velocityError < -1e-5f && previewOffset < reachableOffset - 1e-5f));
    const bool pidLimitBlocks =
        (velocityError > 1e-5f && velocityPreview.unclampedOutput >
            velocityPreview.output + 1e-5f) ||
        (velocityError < -1e-5f && velocityPreview.unclampedOutput <
            velocityPreview.output - 1e-5f);
    const bool pitchOutputLimitBlocks = previewFollowsError &&
        ((velocityError > 1e-5f && pitchPreview.unclampedOutput >
            pitchPreview.output + 1e-5f) ||
         (velocityError < -1e-5f && pitchPreview.unclampedOutput <
            pitchPreview.output - 1e-5f));
    const bool mixerLimitBlocks = previewFollowsError &&
        ((velocityError > 1e-5f && pitchPreview.output >
            mixerPreview.balanceEffort + 1e-5f) ||
         (velocityError < -1e-5f && pitchPreview.output <
            mixerPreview.balanceEffort - 1e-5f));
    const bool suppressIntegral = pitchLimitBlocks || pidLimitBlocks ||
        pitchOutputLimitBlocks || mixerLimitBlocks;

    const auto velocityStep = m_velocityPid.computeDetailed(
        targetVelocity,
        input.odometry.velocityMps,
        input.dt,
        !suppressIntegral);
    if (!velocityStep.valid) {
        return failMotion();
    }

    const float requestedCorrection = velocityStep.output;
    const float boundedCorrection = clampSymmetric(
        requestedCorrection, m_config.max_pitch_offset_deg);
    const float requestedPitch = clampTargetPitch(
        m_config.pitch_trim_deg + boundedCorrection);
    const float targetPitch = slewTargetPitch(requestedPitch, input.dt);
    m_velocity_anti_windup = suppressIntegral;

    const auto pitchStep = m_pitchPid.computeWithMeasurementRateDetailed(
        targetPitch,
        input.currentPitch_deg,
        input.currentPitchRate_dps,
        input.dt);
    if (!pitchStep.valid) {
        return failMotion();
    }

    const LongitudinalMixerResult mixed = mixEfforts(
        pitchStep.output,
        0.0f,
        m_config.max_effort,
        m_config.sync_max_effort);

    m_last_target_pitch_deg = targetPitch;
    m_last_target_velocity_mps = targetVelocity;
    m_last_command_velocity_mps = commandVelocity;
    m_last_speed_setpoint_left_dps = 0.0f;
    m_last_speed_setpoint_right_dps = 0.0f;
    m_last_target_yaw_deg = std::isfinite(input.currentYaw_deg)
        ? input.currentYaw_deg : 0.0f;
    m_last_desired_yaw_rate_dps = 0.0f;

    m_last_diagnostics.phase = m_motion_phase;
    m_last_diagnostics.valid = true;
    m_last_diagnostics.targetPitchValid = true;
    m_last_diagnostics.targetPitchClamped =
        requestedPitch != (m_config.pitch_trim_deg + requestedCorrection);
    m_last_diagnostics.targetPitchRateLimited = targetPitch != requestedPitch;
    m_last_diagnostics.positionLoopEnabled = false;
    m_last_diagnostics.velocityLoopEnabled = true;
    m_last_diagnostics.synchronizationEnabled = false;
    m_last_diagnostics.motionCommandFresh = commandFresh;
    m_last_diagnostics.velocityFeedbackValid = true;
    m_last_diagnostics.velocityTargetClamped =
        commandFinite && boundedCommand != commandVelocity;
    m_last_diagnostics.velocityOutputSaturated = velocityStep.saturated ||
        boundedCorrection != requestedCorrection;
    m_last_diagnostics.velocityAntiWindup = m_velocity_anti_windup;
    m_last_diagnostics.targetPitch_deg = targetPitch;
    m_last_diagnostics.targetVelocityMps = targetVelocity;
    m_last_diagnostics.commandVelocityMps = commandVelocity;
    m_last_diagnostics.measuredVelocityMps = input.odometry.velocityMps;
    m_last_diagnostics.velocityCorrection_deg = boundedCorrection;
    m_last_diagnostics.holdPositionM = m_hold_position_m;
    m_last_diagnostics.balanceSaturated = mixed.balanceSaturated;
    m_last_diagnostics.syncLimited = mixed.syncLimited;
    m_last_diagnostics.requestedBalanceEffort = pitchStep.output;
    m_last_diagnostics.balanceEffort = mixed.balanceEffort;
    m_last_diagnostics.requestedSyncEffort = 0.0f;
    m_last_diagnostics.syncEffort = mixed.syncEffort;
    m_last_diagnostics.leftEffort = mixed.left;
    m_last_diagnostics.rightEffort = mixed.right;

    return {mixed.left, mixed.right};
}

void LongitudinalCascadeBalanceStrategy::reset()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pitchPid.reset();
    m_velocityPid.reset();
    resetMotionState();
    m_targetPitch_deg = m_config.pitch_trim_deg;
    m_last_target_pitch_deg = m_config.pitch_trim_deg;
    m_last_speed_setpoint_left_dps = 0.0f;
    m_last_speed_setpoint_right_dps = 0.0f;
    m_last_target_yaw_deg = 0.0f;
    m_last_desired_yaw_rate_dps = 0.0f;
    m_last_diagnostics = {};
    m_last_diagnostics.strategyId = BalanceStrategyId::LONGITUDINAL_CASCADE;
    m_last_diagnostics.phase = m_config.configured
        ? BalanceControlPhase::PITCH_BASELINE
        : BalanceControlPhase::INACTIVE;
    m_target_pitch_initialized = false;
}

void LongitudinalCascadeBalanceStrategy::applyConfig(const ConfigData& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_config = config.control.strategies.longitudinal_cascade;
    m_pitchPid.updateParams(m_config.pitch);
    m_velocityPid.updateParams(m_config.velocity);
    m_motionProfile.configure({
        m_config.max_velocity_mps,
        m_config.max_acceleration_mps2,
        m_config.max_deceleration_mps2
    });
    resetMotionState();
    m_targetPitch_deg = m_config.pitch_trim_deg;
    m_last_target_pitch_deg = m_config.pitch_trim_deg;
    m_last_diagnostics = {};
    m_last_diagnostics.strategyId = BalanceStrategyId::LONGITUDINAL_CASCADE;
    m_last_diagnostics.phase = m_config.configured
        ? BalanceControlPhase::PITCH_BASELINE
        : BalanceControlPhase::INACTIVE;
    m_target_pitch_initialized = false;
}

void LongitudinalCascadeBalanceStrategy::updatePidConfig(const std::string& pidName,
                                                         const PIDConfig& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (pidName == "pitch") {
        m_config.pitch = config;
        m_pitchPid.updateParams(config);
    } else if (pidName == "velocity") {
        m_config.velocity = config;
        m_velocityPid.updateParams(config);
    }
}

float LongitudinalCascadeBalanceStrategy::getLastSpeedSetpointLeftDPS() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_speed_setpoint_left_dps;
}

float LongitudinalCascadeBalanceStrategy::getLastSpeedSetpointRightDPS() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_speed_setpoint_right_dps;
}

float LongitudinalCascadeBalanceStrategy::getLastTargetYawDeg() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_target_yaw_deg;
}

float LongitudinalCascadeBalanceStrategy::getLastDesiredYawRateDPS() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_desired_yaw_rate_dps;
}

BalanceControlDiagnostics LongitudinalCascadeBalanceStrategy::getDiagnostics() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_diagnostics;
}

float LongitudinalCascadeBalanceStrategy::getLastTargetPitchDeg() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_target_pitch_deg;
}

float LongitudinalCascadeBalanceStrategy::getLastTargetVelocityMps() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_target_velocity_mps;
}

double LongitudinalCascadeBalanceStrategy::getLastHoldPositionM() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_hold_position_m;
}

LongitudinalMixerResult LongitudinalCascadeBalanceStrategy::mixEfforts(float balanceRequested,
                                                                       float syncRequested,
                                                                       float maxEffort,
                                                                       float syncLimit)
{
    LongitudinalMixerResult result = {};
    if (!std::isfinite(balanceRequested) || !std::isfinite(syncRequested) ||
        !std::isfinite(maxEffort) || !std::isfinite(syncLimit) || maxEffort <= 0.0f) {
        return result;
    }

    const float balance = std::max(-maxEffort, std::min(maxEffort, balanceRequested));
    result.balanceEffort = balance;
    result.balanceSaturated = balance != balanceRequested;

    const float availableSync = std::max(0.0f, maxEffort - std::fabs(balance));
    const float allowedSync = std::min(std::max(0.0f, syncLimit), availableSync);
    result.syncEffort = std::max(-allowedSync, std::min(allowedSync, syncRequested));
    result.syncLimited = result.syncEffort != syncRequested;

    result.left = std::max(-maxEffort, std::min(maxEffort, balance - result.syncEffort));
    result.right = std::max(-maxEffort, std::min(maxEffort, balance + result.syncEffort));
    return result;
}

float LongitudinalCascadeBalanceStrategy::clampTargetPitch(float targetPitch_deg) const
{
    if (!std::isfinite(targetPitch_deg) || !std::isfinite(m_config.pitch_trim_deg)) {
        return m_config.pitch_trim_deg;
    }
    if (m_config.max_pitch_offset_deg <= 0.0f ||
        !std::isfinite(m_config.max_pitch_offset_deg)) {
        return m_config.pitch_trim_deg;
    }
    const float offset = targetPitch_deg - m_config.pitch_trim_deg;
    return m_config.pitch_trim_deg + clampSymmetric(offset, m_config.max_pitch_offset_deg);
}

float LongitudinalCascadeBalanceStrategy::slewTargetPitch(float targetPitch_deg, float dtSeconds)
{
    if (!m_target_pitch_initialized) {
        m_targetPitch_deg = m_config.pitch_trim_deg;
        m_target_pitch_initialized = true;
    }
    const float maxRate = m_config.max_pitch_rate_dps;
    if (!std::isfinite(maxRate) || maxRate <= 0.0f) {
        m_targetPitch_deg = targetPitch_deg;
        return m_targetPitch_deg;
    }

    const float maxStep = maxRate * dtSeconds;
    const float delta = targetPitch_deg - m_targetPitch_deg;
    m_targetPitch_deg += clampSymmetric(delta, maxStep);
    return m_targetPitch_deg;
}

void LongitudinalCascadeBalanceStrategy::resetMotionState()
{
    m_motionProfile.reset();
    m_motion_session_initialized = false;
    m_velocity_anti_windup = false;
    m_motion_phase = BalanceControlPhase::INACTIVE;
    m_hold_position_m = 0.0;
    m_last_target_velocity_mps = 0.0f;
    m_last_command_velocity_mps = 0.0f;
}
