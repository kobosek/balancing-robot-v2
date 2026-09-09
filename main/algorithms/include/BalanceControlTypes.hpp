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
    CAPTURE,
    HOLD,
    DRIVE,
    BRAKE,
    FAULT
};

// The command is produced by CommandProcessor and consumed as one coherent
// snapshot by RobotController.  `fresh` is refreshed for every received
// joystick packet, even when its numeric value did not change.
struct LongitudinalMotionCommand {
    bool valid = false;
    bool fresh = true;
    bool stop = true;
    float targetVelocityMps = 0.0f;
    uint64_t sequence = 0;
    int64_t receivedTimestampUs = 0;
    uint64_t armId = 0;
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
    bool motionCommandValid = false;
    bool motionCommandFresh = false;
    bool velocityFeedbackValid = false;
    bool velocityTargetClamped = false;
    bool velocityOutputSaturated = false;
    bool velocityAntiWindup = false;
    float targetPitch_deg = 0.0f;
    float targetVelocityMps = 0.0f;
    float commandVelocityMps = 0.0f;
    float measuredVelocityMps = 0.0f;
    float velocityCorrection_deg = 0.0f;
    double holdPositionM = 0.0;
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
    int64_t nowUs = 0;
    int64_t motionTimeoutUs = 0;
    LongitudinalMotionCommand motion = {};
    // Longitudinal data is optional for the legacy NestedPid strategy. The
    // complete sample is carried together so a future strategy cannot mix
    // wheel counts from one frame with speeds from another.
    LongitudinalOdometryResult odometry = {};
};
