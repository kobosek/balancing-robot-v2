#include "LongitudinalCascadeBalanceStrategy.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>

namespace {
float clampSymmetric(float value, float limit)
{
    if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0f) {
        return 0.0f;
    }
    return std::max(-limit, std::min(limit, value));
}

float clampUnit(float value)
{
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, value));
}

float velocityOutsideDeadband(float velocity, float deadband)
{
    if (!std::isfinite(velocity) || !std::isfinite(deadband) || deadband < 0.0f) {
        return std::numeric_limits<float>::infinity();
    }
    return std::max(0.0f, std::fabs(velocity) - deadband);
}

double removeDeadband(double value, double deadband)
{
    if (!std::isfinite(value) || !std::isfinite(deadband) || deadband < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double magnitude = std::fabs(value);
    if (magnitude <= deadband) {
        return 0.0;
    }
    return std::copysign(magnitude - deadband, value);
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
    m_last_diagnostics.loopMode = static_cast<int8_t>(m_config.loop_mode);
    m_last_diagnostics.phase = m_config.configured
        ? BalanceControlPhase::PITCH_BASELINE
        : BalanceControlPhase::INACTIVE;
    m_last_diagnostics.odometryGeneration = input.odometry.generation;
    m_last_diagnostics.odometrySequence = input.odometry.odometrySequence;
    if (!m_config.configured || !std::isfinite(input.dt) || input.dt <= 0.0f ||
        input.dt > MAX_CONTROL_DT_SECONDS ||
        !std::isfinite(input.currentPitch_deg) ||
        !std::isfinite(input.currentPitchRate_dps)) {
        return {};
    }

    switch (m_config.loop_mode) {
        case LongitudinalLoopMode::PITCH_ONLY:
            return updatePitchBaseline(input);
        case LongitudinalLoopMode::VELOCITY:
            return updateMotion(input);
        case LongitudinalLoopMode::POSITION_HOLD:
            return updateMotion(input);
        default:
            m_last_diagnostics.phase = BalanceControlPhase::FAULT;
            return {};
    }
}

MotorEffort LongitudinalCascadeBalanceStrategy::updatePitchBaseline(
    const BalanceControlInput& input)
{
    if (!std::isfinite(input.targetPitchOffset_deg)) {
        return {};
    }

    if (m_motion_arm_initialized || m_motion_session_initialized ||
        m_motion_phase == BalanceControlPhase::DRIVE ||
        m_motion_phase == BalanceControlPhase::BRAKE ||
        m_motion_phase == BalanceControlPhase::HOLD ||
        m_motion_phase == BalanceControlPhase::CAPTURE) {
        resetMotionState();
    }

    const float requestedPitch = clampTargetPitch(
        m_config.pitch_trim_deg + input.targetPitchOffset_deg);
    // Keep the slew state transactional as well: a finite sample can still
    // produce an invalid PID intermediate (for example after a bad runtime
    // parameter update), and that sample must not leave a new target behind.
    const float currentTargetPitch = m_target_pitch_initialized
        ? m_targetPitch_deg : m_config.pitch_trim_deg;
    const float targetPitch = [&] {
        if (!std::isfinite(m_config.max_pitch_rate_dps) ||
            m_config.max_pitch_rate_dps <= 0.0f) {
            return requestedPitch;
        }
        const float maxStep = m_config.max_pitch_rate_dps * input.dt;
        if (!std::isfinite(maxStep) || maxStep <= 0.0f) {
            return currentTargetPitch;
        }
        return currentTargetPitch + clampSymmetric(
            requestedPitch - currentTargetPitch, maxStep);
    }();
    const auto pitchPreview = m_pitchPid.previewWithMeasurementRate(
        targetPitch,
        input.currentPitch_deg,
        input.currentPitchRate_dps,
        input.dt);
    if (!pitchPreview.valid) {
        return {};
    }
    m_targetPitch_deg = targetPitch;
    m_target_pitch_initialized = true;
    const auto pitchStep = m_pitchPid.computeWithMeasurementRateDetailed(
        targetPitch,
        input.currentPitch_deg,
        input.currentPitchRate_dps,
        input.dt);
    if (!pitchStep.valid) {
        m_pitchPid.reset();
        m_targetPitch_deg = m_config.pitch_trim_deg;
        m_target_pitch_initialized = false;
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
    m_last_diagnostics.balanceSaturated = pitchStep.saturated || mixed.balanceSaturated;
    m_last_diagnostics.pitchPidSaturated = pitchStep.saturated;
    m_last_diagnostics.mixerSaturated = mixed.balanceSaturated ||
        mixed.syncLimited || mixed.wheelLimited;
    m_last_diagnostics.syncLimited = mixed.syncLimited;
    m_last_diagnostics.targetPitch_deg = targetPitch;
    m_last_diagnostics.requestedBalanceEffort = pitchStep.unclampedOutput;
    m_last_diagnostics.balanceEffort = mixed.balanceEffort;
    m_last_diagnostics.requestedSyncEffort = 0.0f;
    m_last_diagnostics.syncEffort = mixed.syncEffort;
    m_last_diagnostics.leftEffort = mixed.left;
    m_last_diagnostics.rightEffort = mixed.right;
    m_last_balance_effort = mixed.balanceEffort;
    m_last_left_effort = mixed.left;
    m_last_right_effort = mixed.right;

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
        m_pitchPid.reset();
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

    const bool commandSessionValid = !input.motion.valid ||
        input.motion.armId == input.controlArmId;
    m_last_diagnostics.motionCommandValid = input.motion.valid && commandSessionValid;
    m_last_diagnostics.motionCommandFresh = false;
    m_last_diagnostics.commandVelocityMps = input.motion.valid &&
        std::isfinite(input.motion.targetVelocityMps)
        ? input.motion.targetVelocityMps : 0.0f;
    m_last_diagnostics.holdPositionM = m_hold_position_m;
    m_last_diagnostics.odometryGeneration = input.odometry.generation;
    m_last_diagnostics.odometrySequence = input.odometry.odometrySequence;

    // A duplicate control call may reuse the last coherent encoder sample
    // while it is still fresh. An out-of-order or invalid frame must never
    // masquerade as that sample through copied diagnostic fields.
    if ((input.odometry.status != LongitudinalOdometryUpdateStatus::ACCEPTED &&
         input.odometry.status != LongitudinalOdometryUpdateStatus::DUPLICATE) ||
        !input.odometry.odometryValid ||
        !input.odometry.positionValid ||
        !input.odometry.velocityValid ||
        !input.odometry.leftFeedbackValid ||
        !input.odometry.rightFeedbackValid ||
        !std::isfinite(input.odometry.positionM) ||
        !std::isfinite(input.odometry.distanceDifferenceM) ||
        !std::isfinite(input.odometry.leftVelocityMps) ||
        !std::isfinite(input.odometry.rightVelocityMps) ||
        !std::isfinite(input.odometry.velocityMps) ||
        !std::isfinite(input.odometry.velocityDifferenceMps) ||
        !std::isfinite(input.currentSpeedLeft_dps) ||
        !std::isfinite(input.currentSpeedRight_dps)) {
        m_velocity_anti_windup = false;
        m_last_diagnostics.velocityFeedbackValid = false;
        return failMotion();
    }

    if (m_odometry_generation_initialized &&
        m_odometry_generation != input.odometry.generation) {
        // A newly recaptured odometry base must not inherit a position or
        // differential-distance target from the previous continuity epoch.
        m_velocity_anti_windup = false;
        return failMotion();
    }
    if (!m_odometry_generation_initialized) {
        m_odometry_generation = input.odometry.generation;
        m_odometry_generation_initialized = true;
    }

    if (m_motion_arm_initialized && m_motion_arm_id != input.controlArmId) {
        // Arm changes start a new motion session. No profile, HOLD target or
        // integral from the previous motor authorization may cross that
        // boundary.
        m_velocityPid.reset();
        resetMotionState();
    }
    if (!m_motion_arm_initialized) {
        m_motion_arm_id = input.controlArmId;
        m_motion_arm_initialized = true;
    }

    if (!m_motion_session_initialized) {
        m_motionProfile.reset();
        m_motion_session_initialized = true;
        m_motion_phase = BalanceControlPhase::CAPTURE;
        m_hold_position_m = input.odometry.positionM;
        m_velocity_anti_windup = false;
        m_velocityPid.reset();
    }

    bool commandFresh = input.motion.valid && commandSessionValid && input.motion.fresh;
    if (input.motion.valid && input.nowUs > 0 && input.motionTimeoutUs > 0) {
        if (input.motion.receivedTimestampUs <= 0) {
            commandFresh = false;
        } else {
            const int64_t ageUs = input.nowUs - input.motion.receivedTimestampUs;
            commandFresh = commandFresh && ageUs >= 0 && ageUs <= input.motionTimeoutUs;
        }
    }

    const float commandVelocity = input.motion.valid
        ? input.motion.targetVelocityMps : 0.0f;
    const bool commandFinite = !input.motion.valid || std::isfinite(commandVelocity);
    if (!commandFinite) {
        return failMotion();
    }
    const float boundedCommand = commandFinite
        ? clampSymmetric(commandVelocity, m_config.max_velocity_mps)
        : 0.0f;
    const bool commandActive = commandFresh && !input.motion.stop &&
        commandFinite && std::fabs(boundedCommand) > 1e-5f;
    const float requestScale = commandActive
        ? computeMotionRequestScale(input) : 1.0f;
    const float profiledCommand = commandActive
        ? boundedCommand * requestScale : 0.0f;
    const bool driveRequested = commandActive &&
        std::fabs(profiledCommand) > 1e-5f;

    const BalanceControlPhase previousPhase = m_motion_phase;
    const bool positionMode =
        m_config.loop_mode == LongitudinalLoopMode::POSITION_HOLD;
    const float holdTargetPitch = m_target_pitch_initialized
        ? m_targetPitch_deg : m_config.pitch_trim_deg;
    const float holdPitchError = input.currentPitch_deg - holdTargetPitch;
    const float exitPitchError = std::max(
        m_config.hold_pitch_error_deadband_deg * 2.0f,
        m_config.hold_pitch_error_deadband_deg);
    const float exitPitchRate = std::max(
        m_config.hold_pitch_rate_deadband_dps * 2.0f,
        m_config.hold_pitch_rate_deadband_dps);
    const float holdVelocityDeadband = std::max(
        0.0f, m_config.hold_velocity_deadband_mps);
    const bool wheelsStoppedForExit =
        velocityOutsideDeadband(input.odometry.leftVelocityMps,
                                holdVelocityDeadband) <= m_config.hold_exit_velocity_mps &&
        velocityOutsideDeadband(input.odometry.rightVelocityMps,
                                holdVelocityDeadband) <= m_config.hold_exit_velocity_mps;
    const bool attitudeStableForExit =
        std::fabs(holdPitchError) <= exitPitchError &&
        std::fabs(input.currentPitchRate_dps) <= exitPitchRate;
    const bool stableForExit = wheelsStoppedForExit && attitudeStableForExit;

    // Position correction is a P term expressed in m/s.  It is armed only
    // after an established HOLD is settled; once armed it may move the robot
    // through the same profile while the pitch attitude remains stable.
    // DRIVE and BRAKE always use the external command/profile alone.  A
    // two-times release band prevents encoder quantization from chattering
    // the target.
    const double positionErrorM = m_hold_position_m - input.odometry.positionM;
    const float positionDeadband = std::max(0.0f,
                                            m_config.hold_position_deadband_m);
    // The outer position loop uses a conventional Schmitt trigger: enter
    // only outside the upper band and release once back inside the lower
    // band.  Reversing those thresholds makes a small, persistent encoder
    // error alternate the return request on every sample.
    const float positionEnter = std::max(
        positionDeadband * 2.0f, positionDeadband);
    const float positionRelease = positionDeadband;
    bool positionHoldActive = m_position_hold_active;
    if (positionMode && previousPhase == BalanceControlPhase::HOLD &&
        attitudeStableForExit) {
        if (!positionHoldActive &&
            wheelsStoppedForExit && std::fabs(positionErrorM) > positionEnter) {
            positionHoldActive = true;
        } else if (positionHoldActive &&
                   std::fabs(positionErrorM) <= positionRelease) {
            positionHoldActive = false;
        }
    }
    const bool positionCorrectionAllowed = positionMode &&
        previousPhase == BalanceControlPhase::HOLD &&
        attitudeStableForExit && positionHoldActive;
    const double holdVelocityRequestD = positionCorrectionAllowed
        ? static_cast<double>(m_config.position_kp) * positionErrorM
        : 0.0;
    if (!std::isfinite(holdVelocityRequestD) ||
        holdVelocityRequestD > static_cast<double>(FLT_MAX) ||
        holdVelocityRequestD < -static_cast<double>(FLT_MAX)) {
        return failMotion();
    }
    const float holdVelocityRequest = static_cast<float>(holdVelocityRequestD);
    const float holdVelocityTargetBeforeRequestLimit = positionCorrectionAllowed
        ? clampSymmetric(holdVelocityRequest, m_config.max_hold_velocity_mps)
        : 0.0f;
    // The E2 request limiter also gates a position-return request.  A HOLD
    // correction must not regain a large external velocity command's headroom
    // while the robot is already close to its pitch or effort boundary.
    const float positionRequestScale = positionCorrectionAllowed && !commandActive
        ? computeMotionRequestScale(input) : 1.0f;
    const float holdVelocityTarget = holdVelocityTargetBeforeRequestLimit *
        positionRequestScale;
    const bool positionProfileRequested = positionCorrectionAllowed &&
        std::fabs(holdVelocityTarget) > 1e-5f;
    const float profileCommand = driveRequested ? profiledCommand
        : (positionProfileRequested ? holdVelocityTarget : 0.0f);
    const bool profileDriveRequested = driveRequested ||
        positionProfileRequested;

    const float previousProfileVelocity = m_motionProfile.targetVelocityMps();
    const auto profileResult = m_motionProfile.update(
        profileCommand, profileDriveRequested, input.dt);
    if (!profileResult.valid) {
        return failMotion();
    }
    const bool wheelsStoppedForEntry =
        velocityOutsideDeadband(input.odometry.leftVelocityMps,
                                holdVelocityDeadband) <= m_config.hold_enter_velocity_mps &&
        velocityOutsideDeadband(input.odometry.rightVelocityMps,
                                holdVelocityDeadband) <= m_config.hold_enter_velocity_mps;
    const bool attitudeSettledForEntry =
        std::fabs(holdPitchError) <= m_config.hold_pitch_error_deadband_deg &&
        std::fabs(input.currentPitchRate_dps) <= m_config.hold_pitch_rate_deadband_dps;
    const bool stableForEntry = profileResult.atRest &&
        wheelsStoppedForEntry && attitudeSettledForEntry;
    const float settleTimeSeconds = std::max(
        0.0f, static_cast<float>(m_config.hold_settle_time_ms) / 1000.0f);

    if (driveRequested) {
        m_motion_phase = BalanceControlPhase::DRIVE;
        m_hold_settle_elapsed_s = 0.0f;
        m_hold_stable_observation_active = false;
    } else if (positionMode && previousPhase == BalanceControlPhase::HOLD &&
               attitudeStableForExit &&
               (positionHoldActive || stableForExit)) {
        // A position correction is still part of HOLD.  Its profiled velocity
        // target must not turn the phase into DRIVE or repeatedly recapture
        // the stop position.
        m_motion_phase = BalanceControlPhase::HOLD;
        m_hold_settle_elapsed_s = settleTimeSeconds;
    } else if (!profileResult.atRest) {
        m_motion_phase = BalanceControlPhase::BRAKE;
        m_hold_settle_elapsed_s = 0.0f;
    } else if (previousPhase == BalanceControlPhase::HOLD) {
        // Velocity hysteresis keeps small encoder noise from repeatedly
        // leaving and re-entering HOLD. Attitude thresholds use the same
        // policy with a wider exit band because no separate pitch setting is
        // needed at this stage.
        if (stableForExit) {
            m_motion_phase = BalanceControlPhase::HOLD;
        } else {
            m_motion_phase = BalanceControlPhase::BRAKE;
            m_hold_settle_elapsed_s = 0.0f;
            m_hold_stable_observation_active = false;
        }
    } else {
        if (stableForEntry) {
            // The first stable frame starts the observation window. Its dt
            // precedes the observation and must not be counted as settled
            // time. This also gives settle=0 an explicit stability guard.
            if (!m_hold_stable_observation_active) {
                m_hold_stable_observation_active = true;
                m_hold_settle_elapsed_s = 0.0f;
            } else {
                m_hold_settle_elapsed_s = std::min(
                    settleTimeSeconds,
                    m_hold_settle_elapsed_s + input.dt);
            }
        } else {
            m_hold_settle_elapsed_s = 0.0f;
            m_hold_stable_observation_active = false;
        }
        m_motion_phase = stableForEntry &&
            ((settleTimeSeconds <= 0.0f) ||
             m_hold_settle_elapsed_s >= settleTimeSeconds)
            ? BalanceControlPhase::HOLD : BalanceControlPhase::BRAKE;
    }

    const bool enteredHold = previousPhase != BalanceControlPhase::HOLD &&
        m_motion_phase == BalanceControlPhase::HOLD;
    if (enteredHold) {
        // Capture the position at the end of braking.  The position P loop is
        // enabled only on the following settled HOLD sample, so this frame
        // cannot immediately chase a moving target.
        m_hold_position_m = input.odometry.positionM;
        m_position_hold_active = false;
    } else if (m_motion_phase == BalanceControlPhase::HOLD && positionMode &&
               previousPhase == BalanceControlPhase::HOLD &&
               attitudeStableForExit) {
        m_position_hold_active = positionHoldActive;
    } else if (m_motion_phase != BalanceControlPhase::HOLD) {
        m_position_hold_active = false;
    }

    const bool reversalReachedZero = driveRequested &&
        previousPhase == BalanceControlPhase::DRIVE &&
        std::fabs(previousProfileVelocity) > 1e-5f &&
        profileResult.atRest &&
        std::signbit(previousProfileVelocity) != std::signbit(profileCommand);
    if (driveRequested && (previousPhase != BalanceControlPhase::DRIVE ||
                           reversalReachedZero)) {
        // One differential-distance reference per DRIVE segment.  It remains
        // valid through BRAKE and HOLD, and is replaced only by a later
        // segment or a new arm.
        m_distance_difference_target_m = input.odometry.distanceDifferenceM;
        m_distance_target_valid = true;
    }

    const bool synchronizationEnabled = m_config.sync_enabled &&
        m_distance_target_valid &&
        (m_motion_phase == BalanceControlPhase::DRIVE ||
         m_motion_phase == BalanceControlPhase::BRAKE ||
         m_motion_phase == BalanceControlPhase::HOLD);
    const double distanceErrorForSync = synchronizationEnabled
        ? removeDeadband(
            input.odometry.distanceDifferenceM -
                m_distance_difference_target_m,
            m_config.sync_position_deadband_m)
        : 0.0;
    const double velocityDifferenceForSync = synchronizationEnabled
        ? removeDeadband(
            input.odometry.velocityDifferenceMps,
            m_config.sync_velocity_deadband_mps)
        : 0.0;
    const double velocitySyncRequestedD =
        static_cast<double>(m_config.sync_kd) * velocityDifferenceForSync;
    const double positionSyncRequestedD =
        static_cast<double>(m_config.sync_kp) * distanceErrorForSync;
    const double syncRequestedD = velocitySyncRequestedD +
        positionSyncRequestedD;
    if (!std::isfinite(syncRequestedD) ||
        syncRequestedD > static_cast<double>(FLT_MAX) ||
        syncRequestedD < -static_cast<double>(FLT_MAX)) {
        return failMotion();
    }
    const float syncRequested = static_cast<float>(syncRequestedD);

    const float targetVelocity = profileResult.targetVelocityMps;
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
        syncRequested,
        m_config.max_effort,
        m_config.sync_max_effort);
    // Anti-windup follows the candidate trapezoidal integral change, rather
    // than the instantaneous error.  This matters when the error changes
    // sign while the candidate integral is still moving toward a limit.
    const bool integralPushesPositive = velocityPreview.integralDelta > 1e-5f;
    const bool integralPushesNegative = velocityPreview.integralDelta < -1e-5f;
    const bool correctionLimitBlocks =
        (integralPushesPositive && previewCorrection > previewBoundedCorrection + 1e-5f) ||
        (integralPushesNegative && previewCorrection < previewBoundedCorrection - 1e-5f);
    const bool pitchSlewBlocks =
        (integralPushesPositive && previewOffset > reachableOffset + 1e-5f) ||
        (integralPushesNegative && previewOffset < reachableOffset - 1e-5f);
    const bool pidLimitBlocks =
        (integralPushesPositive && velocityPreview.unclampedOutput >
            velocityPreview.output + 1e-5f) ||
        (integralPushesNegative && velocityPreview.unclampedOutput <
            velocityPreview.output - 1e-5f);
    const bool pitchOutputLimitBlocks =
        (integralPushesPositive && pitchPreview.unclampedOutput >
            pitchPreview.output + 1e-5f) ||
        (integralPushesNegative && pitchPreview.unclampedOutput <
            pitchPreview.output - 1e-5f);
    const bool mixerLimitBlocks =
        (integralPushesPositive && pitchPreview.output >
            mixerPreview.balanceEffort + 1e-5f) ||
        (integralPushesNegative && pitchPreview.output <
            mixerPreview.balanceEffort - 1e-5f);
    const bool suppressIntegral = correctionLimitBlocks || pitchSlewBlocks ||
        pidLimitBlocks || pitchOutputLimitBlocks || mixerLimitBlocks;

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
    m_velocity_anti_windup = suppressIntegral &&
        std::fabs(velocityPreview.integralDelta) > 1e-5f;

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
        syncRequested,
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
    const bool positionTargetValid = positionMode &&
        m_motion_phase == BalanceControlPhase::HOLD;
    m_last_diagnostics.positionLoopEnabled = positionTargetValid;
    m_last_diagnostics.velocityLoopEnabled = true;
    m_last_diagnostics.synchronizationEnabled = synchronizationEnabled;
    m_last_diagnostics.motionCommandFresh = commandFresh;
    m_last_diagnostics.velocityFeedbackValid = true;
    m_last_diagnostics.velocityTargetClamped =
        commandFinite && (boundedCommand != commandVelocity ||
                          profiledCommand != boundedCommand ||
                          holdVelocityTarget != holdVelocityRequest);
    m_last_diagnostics.motionRequestLimited =
        (commandActive && requestScale < 1.0f - 1e-5f) ||
        (positionCorrectionAllowed && positionRequestScale < 1.0f - 1e-5f);
    m_last_diagnostics.velocityOutputSaturated = velocityStep.saturated ||
        boundedCorrection != requestedCorrection;
    m_last_diagnostics.velocityAntiWindup = m_velocity_anti_windup;
    m_last_diagnostics.targetPitch_deg = targetPitch;
    m_last_diagnostics.targetVelocityMps = targetVelocity;
    m_last_diagnostics.commandVelocityMps = commandVelocity;
    m_last_diagnostics.measuredVelocityMps = input.odometry.velocityMps;
    m_last_diagnostics.velocityCorrection_deg = boundedCorrection;
    m_last_diagnostics.holdPositionM = m_hold_position_m;
    m_last_diagnostics.positionTargetValid = positionTargetValid;
    m_last_diagnostics.positionHoldActive = positionTargetValid &&
        m_position_hold_active;
    m_last_diagnostics.synchronizationTargetValid = m_distance_target_valid;
    m_last_diagnostics.holdVelocityRequestMps = holdVelocityRequest;
    m_last_diagnostics.holdVelocityTargetMps = holdVelocityTarget;
    m_last_diagnostics.syncVelocityDifferenceMps =
        input.odometry.velocityDifferenceMps;
    m_last_diagnostics.positionM = input.odometry.positionM;
    m_last_diagnostics.positionErrorM =
        m_hold_position_m - input.odometry.positionM;
    m_last_diagnostics.distanceDifferenceM =
        input.odometry.distanceDifferenceM;
    m_last_diagnostics.distanceDifferenceTargetM =
        m_distance_difference_target_m;
    m_last_diagnostics.balanceSaturated = pitchStep.saturated || mixed.balanceSaturated;
    m_last_diagnostics.pitchPidSaturated = pitchStep.saturated;
    m_last_diagnostics.mixerSaturated = mixed.balanceSaturated ||
        mixed.syncLimited || mixed.wheelLimited;
    m_last_diagnostics.syncLimited = mixed.syncLimited;
    m_last_diagnostics.requestedBalanceEffort = pitchStep.unclampedOutput;
    m_last_diagnostics.balanceEffort = mixed.balanceEffort;
    m_last_diagnostics.requestedSyncEffort = syncRequested;
    m_last_diagnostics.syncEffort = mixed.syncEffort;
    m_last_diagnostics.leftEffort = mixed.left;
    m_last_diagnostics.rightEffort = mixed.right;
    m_last_balance_effort = mixed.balanceEffort;
    m_last_left_effort = mixed.left;
    m_last_right_effort = mixed.right;

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

float LongitudinalCascadeBalanceStrategy::computeMotionRequestScale(
    const BalanceControlInput& input)
{
    const auto& limit = m_config;
    if (!limit.motion_request_limit_enabled ||
        !std::isfinite(input.currentPitch_deg) ||
        !std::isfinite(limit.pitch_trim_deg)) {
        m_motion_request_limited = false;
        return 1.0f;
    }

    const float pitchDistance = std::fabs(
        input.currentPitch_deg - limit.pitch_trim_deg);
    const float previousEffort = std::max(
        std::fabs(m_last_balance_effort),
        std::max(std::fabs(m_last_left_effort),
                 std::fabs(m_last_right_effort)));
    const float effortRatio = limit.max_effort > 0.0f &&
        std::isfinite(limit.max_effort)
        ? clampUnit(previousEffort / limit.max_effort) : 0.0f;

    const bool enters = pitchDistance >= limit.motion_request_limit_pitch_start_deg ||
        effortRatio >= limit.motion_request_limit_effort_start;
    const bool releases = pitchDistance <= limit.motion_request_limit_pitch_release_deg &&
        effortRatio <= limit.motion_request_limit_effort_release;
    if (!m_motion_request_limited && enters) {
        m_motion_request_limited = true;
    } else if (m_motion_request_limited && releases) {
        m_motion_request_limited = false;
    }
    if (!m_motion_request_limited) {
        return 1.0f;
    }

    const float minScale = clampUnit(limit.motion_request_limit_min_scale);
    const auto metricScale = [minScale](float value, float release, float full) {
        if (value <= release) return 1.0f;
        if (value >= full) return minScale;
        const float span = full - release;
        if (!std::isfinite(span) || span <= 0.0f) {
            return minScale;
        }
        const float fraction = clampUnit((value - release) / span);
        return 1.0f - fraction * (1.0f - minScale);
    };

    return std::min(
        metricScale(pitchDistance,
                    limit.motion_request_limit_pitch_release_deg,
                    limit.motion_request_limit_pitch_full_deg),
        metricScale(effortRatio,
                    limit.motion_request_limit_effort_release,
                    limit.motion_request_limit_effort_full));
}

void LongitudinalCascadeBalanceStrategy::applyConfig(const ConfigData& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto& next = config.control.strategies.longitudinal_cascade;
    auto previous = m_config;
    previous.revision = next.revision;
    const bool firstConfig = !m_has_config;
    const bool pitchChanged = firstConfig || previous.pitch != next.pitch;
    const bool velocityChanged = firstConfig || previous.velocity != next.velocity;
    const bool profileChanged = firstConfig ||
        previous.max_velocity_mps != next.max_velocity_mps ||
        previous.max_acceleration_mps2 != next.max_acceleration_mps2 ||
        previous.max_deceleration_mps2 != next.max_deceleration_mps2;
    const bool motionStateChanged = firstConfig ||
        previous.loop_mode != next.loop_mode ||
        previous.pitch_trim_deg != next.pitch_trim_deg ||
        previous.max_pitch_offset_deg != next.max_pitch_offset_deg ||
        previous.max_pitch_rate_dps != next.max_pitch_rate_dps ||
        profileChanged || previous.hold_enter_velocity_mps != next.hold_enter_velocity_mps ||
        previous.hold_exit_velocity_mps != next.hold_exit_velocity_mps ||
        previous.hold_pitch_error_deadband_deg != next.hold_pitch_error_deadband_deg ||
        previous.hold_pitch_rate_deadband_dps != next.hold_pitch_rate_deadband_dps ||
        previous.hold_settle_time_ms != next.hold_settle_time_ms ||
        previous.position_kp != next.position_kp ||
        previous.max_hold_velocity_mps != next.max_hold_velocity_mps ||
        previous.hold_position_deadband_m != next.hold_position_deadband_m ||
        previous.hold_velocity_deadband_mps != next.hold_velocity_deadband_mps ||
        previous.sync_enabled != next.sync_enabled ||
        previous.sync_kp != next.sync_kp ||
        previous.sync_kd != next.sync_kd ||
        previous.sync_position_deadband_m != next.sync_position_deadband_m ||
        previous.sync_velocity_deadband_mps != next.sync_velocity_deadband_mps ||
        previous.sync_max_effort != next.sync_max_effort ||
        previous.motion_request_limit_enabled != next.motion_request_limit_enabled ||
        previous.motion_request_limit_pitch_start_deg != next.motion_request_limit_pitch_start_deg ||
        previous.motion_request_limit_pitch_full_deg != next.motion_request_limit_pitch_full_deg ||
        previous.motion_request_limit_pitch_release_deg != next.motion_request_limit_pitch_release_deg ||
        previous.motion_request_limit_effort_start != next.motion_request_limit_effort_start ||
        previous.motion_request_limit_effort_full != next.motion_request_limit_effort_full ||
        previous.motion_request_limit_effort_release != next.motion_request_limit_effort_release ||
        previous.motion_request_limit_min_scale != next.motion_request_limit_min_scale ||
        previous.configured != next.configured;

    m_config = next;
    m_has_config = true;
    if (pitchChanged) {
        m_pitchPid.updateParams(m_config.pitch);
    }
    if (velocityChanged) {
        m_velocityPid.updateParams(m_config.velocity);
    }
    if (profileChanged) {
        m_motionProfile.configure({
            m_config.max_velocity_mps,
            m_config.max_acceleration_mps2,
            m_config.max_deceleration_mps2
        });
    }
    if (motionStateChanged) {
        resetMotionState();
        // A change of loop level, profile or pitch limits starts a new motion
        // session.  Do not carry the velocity integral into the new cascade;
        // HOLD itself deliberately does preserve that integral.
        if (!velocityChanged) {
            m_velocityPid.reset();
        }
        m_targetPitch_deg = m_config.pitch_trim_deg;
        m_last_target_pitch_deg = m_config.pitch_trim_deg;
        m_target_pitch_initialized = false;
    }
    if (firstConfig || pitchChanged || velocityChanged || motionStateChanged) {
        m_last_diagnostics = {};
        m_last_diagnostics.strategyId = BalanceStrategyId::LONGITUDINAL_CASCADE;
        m_last_diagnostics.phase = m_config.configured
            ? BalanceControlPhase::PITCH_BASELINE
            : BalanceControlPhase::INACTIVE;
    }
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
    const float requestedLeft = balance - result.syncEffort;
    const float requestedRight = balance + result.syncEffort;
    result.left = std::max(-maxEffort, std::min(maxEffort, requestedLeft));
    result.right = std::max(-maxEffort, std::min(maxEffort, requestedRight));
    result.wheelLimited = result.left != requestedLeft || result.right != requestedRight;
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
    m_motion_arm_initialized = false;
    m_motion_arm_id = 0;
    m_velocity_anti_windup = false;
    m_motion_phase = BalanceControlPhase::INACTIVE;
    m_hold_position_m = 0.0;
    m_hold_settle_elapsed_s = 0.0f;
    m_hold_stable_observation_active = false;
    m_last_target_velocity_mps = 0.0f;
    m_last_command_velocity_mps = 0.0f;
    m_motion_request_limited = false;
    m_last_balance_effort = 0.0f;
    m_last_left_effort = 0.0f;
    m_last_right_effort = 0.0f;
    m_position_hold_active = false;
    m_distance_target_valid = false;
    m_distance_difference_target_m = 0.0;
    m_odometry_generation_initialized = false;
    m_odometry_generation = 0;
    // A new arm, mode switch or profile change starts from the configured
    // trim.  The first longitudinal sample will initialize the slew state;
    // never carry the previous target pitch across that boundary.
    m_targetPitch_deg = m_config.pitch_trim_deg;
    m_last_target_pitch_deg = m_config.pitch_trim_deg;
    m_target_pitch_initialized = false;
}
