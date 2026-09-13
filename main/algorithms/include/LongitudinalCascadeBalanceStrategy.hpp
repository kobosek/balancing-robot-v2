#pragma once

#include "IBalanceControlStrategy.hpp"
#include "LongitudinalMotionProfile.hpp"
#include "PIDController.hpp"
#include "config/BalanceStrategyConfig.hpp"
#include <mutex>

struct LongitudinalMixerResult {
    float balanceEffort = 0.0f;
    float syncEffort = 0.0f;
    float left = 0.0f;
    float right = 0.0f;
    bool balanceSaturated = false;
    bool syncLimited = false;
    bool wheelLimited = false;
};

class LongitudinalCascadeBalanceStrategy : public IBalanceControlStrategy {
public:
    LongitudinalCascadeBalanceStrategy();

    const char* name() const override { return "longitudinal_cascade"; }
    MotorEffort update(const BalanceControlInput& input) override;
    void reset() override;
    void applyConfig(const ConfigData& config) override;
    void updatePidConfig(const std::string& pidName, const PIDConfig& config) override;

    float getLastSpeedSetpointLeftDPS() const override;
    float getLastSpeedSetpointRightDPS() const override;
    float getLastTargetYawDeg() const override;
    float getLastDesiredYawRateDPS() const override;
    bool isYawControlEnabled() const override { return false; }
    BalanceControlDiagnostics getDiagnostics() const override;

    // Exposed for deterministic tests and future telemetry.
    float getLastTargetPitchDeg() const;
    float getLastTargetVelocityMps() const;
    double getLastHoldPositionM() const;

    // Keep common and differential effort within the actuator range without
    // changing the requested mean effort.
    static LongitudinalMixerResult mixEfforts(float balanceRequested,
                                              float syncRequested,
                                              float maxEffort,
                                              float syncLimit);

private:
    static constexpr const char* TAG = "LongitudinalCascade";
    static constexpr float MAX_CONTROL_DT_SECONDS = 0.25f;

    mutable std::mutex m_mutex;
    PIDController m_pitchPid;
    PIDController m_velocityPid;
    LongitudinalMotionProfile m_motionProfile;
    LongitudinalCascadeStrategyConfig m_config;
    bool m_has_config = false;

    float m_targetPitch_deg = 0.0f;
    float m_last_target_pitch_deg = 0.0f;
    float m_last_speed_setpoint_left_dps = 0.0f;
    float m_last_speed_setpoint_right_dps = 0.0f;
    float m_last_target_yaw_deg = 0.0f;
    float m_last_desired_yaw_rate_dps = 0.0f;
    BalanceControlDiagnostics m_last_diagnostics = {};
    bool m_target_pitch_initialized = false;
    bool m_motion_session_initialized = false;
    bool m_motion_arm_initialized = false;
    bool m_velocity_anti_windup = false;
    bool m_position_hold_active = false;
    bool m_distance_target_valid = false;
    bool m_odometry_generation_initialized = false;
    BalanceControlPhase m_motion_phase = BalanceControlPhase::INACTIVE;
    double m_hold_position_m = 0.0;
    double m_distance_difference_target_m = 0.0;
    uint32_t m_odometry_generation = 0;
    float m_hold_settle_elapsed_s = 0.0f;
    bool m_hold_stable_observation_active = false;
    uint64_t m_motion_arm_id = 0;
    float m_last_target_velocity_mps = 0.0f;
    float m_last_command_velocity_mps = 0.0f;
    bool m_motion_request_limited = false;
    // update() clears the current-step diagnostics before evaluating the
    // next command. Keep the previous applied effort separately so the
    // effort threshold of the request limiter remains observable.
    float m_last_balance_effort = 0.0f;
    float m_last_left_effort = 0.0f;
    float m_last_right_effort = 0.0f;

    float clampTargetPitch(float targetPitch_deg) const;
    float slewTargetPitch(float targetPitch_deg, float dtSeconds);
    float computeMotionRequestScale(const BalanceControlInput& input);
    MotorEffort updatePitchBaseline(const BalanceControlInput& input);
    MotorEffort updateMotion(const BalanceControlInput& input);
    void resetMotionState();
};
