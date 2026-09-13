#pragma once

#include <stdint.h> // For int64_t

// Define TelemetryDataPoint structure
struct TelemetryDataPoint {
    bool imuValid = false;
    bool encoderLeftValid = false, encoderRightValid = false, imuSampleRepeated = false;
    float imuAgeMs = -1.0f;
    uint32_t imuGeneration = 0;
    int64_t timestamp_us = 0;         // Common
    float pitch_deg = 0.0f;           // Index 0
    float speedLeft_dps = 0.0f;       // Index 1
    float speedRight_dps = 0.0f;      // Index 2
    float batteryVoltage = 0.0f;      // Index 3
    int systemState = 0;              // Index 4
    float speedSetpointLeft_dps = 0.0f; // Index 5
    float speedSetpointRight_dps = 0.0f;// Index 6
    float desiredAngle_deg = 0.0f;    // Index 7
    float yawAngle_deg = 0.0f;        // Index 8
    float targetYawAngle_deg = 0.0f;  // Index 9
    float yawRate_dps = 0.0f;         // Index 10
    float targetYawRate_dps = 0.0f;   // Index 11

    // Version-4 fields are appended after the original 18-element payload.
    // Keep the legacy members and their order stable: older web clients can
    // still consume the first part of a response while new clients use the
    // explicit strategy/control snapshot below.
    uint8_t strategyId = 0;           // Index 19
    int8_t loopMode = -1;             // Index 20; -1 for NestedPid
    uint8_t controlPhase = 0;         // Index 27
    uint8_t phaseReason = 0;          // Index 49
    uint32_t strategyRevision = 0;    // Index 21
    uint32_t configRevision = 0;      // Index 22
    uint64_t commandSessionId = 0;    // Index 23 (serialized as decimal text)
    uint32_t controlGeneration = 0;   // Index 24
    uint32_t odometryGeneration = 0;  // Index 25
    uint64_t odometrySequence = 0;    // Index 26 (serialized as decimal text)

    bool controlValid = false;        // Index 28
    bool targetPitchValid = false;    // Index 29
    bool targetPitchClamped = false;  // Index 30
    bool targetPitchRateLimited = false; // Index 31
    bool positionTargetValid = false; // Index 32
    bool positionHoldActive = false;  // Index 33
    bool synchronizationTargetValid = false; // Index 34
    bool velocityLoopEnabled = false; // Index 35
    bool positionLoopEnabled = false; // Index 36
    bool synchronizationEnabled = false; // Index 37
    bool motionCommandValid = false;  // Index 38
    bool motionCommandFresh = false;  // Index 39
    bool velocityFeedbackValid = false; // Index 40
    bool velocityTargetClamped = false; // Index 41
    bool velocityOutputSaturated = false; // Index 42
    bool velocityAntiWindup = false;  // Index 43
    bool pitchPidSaturated = false;   // Index 44
    bool balanceSaturated = false;    // Index 45
    bool mixerSaturated = false;      // Index 46
    bool syncLimited = false;         // Index 47
    bool motionRequestLimited = false; // Index 48

    float targetPitch_deg = 0.0f;     // Index 50
    float targetVelocityMps = 0.0f;   // Index 51
    float commandVelocityMps = 0.0f;  // Index 52
    float measuredVelocityMps = 0.0f; // Index 53
    float holdVelocityRequestMps = 0.0f; // Index 54
    float holdVelocityTargetMps = 0.0f;  // Index 55
    double positionM = 0.0;           // Index 56
    double holdPositionM = 0.0;       // Index 57
    double positionErrorM = 0.0;      // Index 58
    double distanceDifferenceM = 0.0; // Index 59
    double distanceDifferenceTargetM = 0.0; // Index 60
    float syncVelocityDifferenceMps = 0.0f; // Index 61
    float requestedBalanceEffort = 0.0f; // Index 62
    float balanceEffort = 0.0f;       // Index 63
    float requestedSyncEffort = 0.0f; // Index 64
    float syncEffort = 0.0f;          // Index 65
    float leftEffort = 0.0f;          // Index 66
    float rightEffort = 0.0f;         // Index 67
    bool yawTargetValid = false;      // Index 68
    bool yawControlAvailable = false; // Index 69

    // Optional v4 diagnostics appended after the stable 70-field contract.
    // Older v4 readers may ignore these fields; newer readers use them to
    // distinguish a requested effort from a commit that actually reached the
    // motor driver and to retain the fault context of that step.
    bool motorCommitAttempted = false; // Index 70
    bool motorCommitSucceeded = false; // Index 71
    int32_t motorCommitResult = 0;     // Index 72; esp_err_t-compatible value
    uint8_t faultReason = 0;           // Index 73; 0 means no fault
    bool faultLatched = false;         // Index 74
    uint64_t imuSampleSequence = 0;    // Index 75 (decimal text)
    uint32_t controlStepCostUs = 0;    // Index 76
    bool controlStepLate = false;      // Index 77
};
