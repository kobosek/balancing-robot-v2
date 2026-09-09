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
        !std::isfinite(input.currentPitchRate_dps) ||
        !std::isfinite(input.targetPitchOffset_deg)) {
        return {};
    }

    const float requestedPitch = clampTargetPitch(
        m_config.pitch_trim_deg + input.targetPitchOffset_deg);
    const float targetPitch = slewTargetPitch(requestedPitch, input.dt);
    const float balanceRequested = m_pitchPid.computeWithMeasurementRate(
        targetPitch,
        input.currentPitch_deg,
        input.currentPitchRate_dps,
        input.dt);

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

void LongitudinalCascadeBalanceStrategy::reset()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pitchPid.reset();
    m_velocityPid.reset();
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
