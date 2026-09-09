#pragma once

#include "LongitudinalOdometry.hpp"
#include "config/BalanceStrategyConfig.hpp"
#include <cstdint>

struct MotorEffort {
    float left = 0.0f;
    float right = 0.0f;
};

enum class BalanceControlPhase : uint8_t {
    INACTIVE = 0,
    LEGACY,
    PITCH_BASELINE,
    HOLD,
    DRIVE,
    BRAKE,
    FAULT
};

// Strategy-neutral diagnostics. The payload is intentionally small and
// remains an in-memory control result for now; telemetry serialization is a
// later, versioned stage of the plan.
struct BalanceControlDiagnostics {
    BalanceStrategyId strategyId = BalanceStrategyId::NESTED_PID;
    BalanceControlPhase phase = BalanceControlPhase::INACTIVE;
    bool valid = false;
    bool targetPitchValid = false;
    bool targetPitchClamped = false;
    bool targetPitchRateLimited = false;
    bool balanceSaturated = false;
    bool syncLimited = false;
    bool positionLoopEnabled = false;
    bool velocityLoopEnabled = false;
    bool synchronizationEnabled = false;
    float targetPitch_deg = 0.0f;
    float requestedBalanceEffort = 0.0f;
    float balanceEffort = 0.0f;
    float requestedSyncEffort = 0.0f;
    float syncEffort = 0.0f;
    float leftEffort = 0.0f;
    float rightEffort = 0.0f;
};

struct BalanceControlInput {
    float dt = 0.0f;
    float currentPitch_deg = 0.0f;
    float currentPitchRate_dps = 0.0f;
    float currentYaw_deg = 0.0f;
    float currentYawRate_dps = 0.0f;
    float currentSpeedLeft_dps = 0.0f;
    float currentSpeedRight_dps = 0.0f;
    float targetPitchOffset_deg = 0.0f;
    float targetAngularVelocity_dps = 0.0f;
    // Longitudinal data is optional for the legacy NestedPid strategy. The
    // complete sample is carried together so a future strategy cannot mix
    // wheel counts from one frame with speeds from another.
    LongitudinalOdometryResult odometry = {};
};
