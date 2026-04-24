#pragma once

struct MotorEffort {
    float left = 0.0f;
    float right = 0.0f;
};

struct BalanceControlInput {
    float dt = 0.0f;
    float currentPitch_deg = 0.0f;
    float currentPitchRate_dps = 0.0f;
    float currentYawRate_dps = 0.0f;
    float currentSpeedLeft_dps = 0.0f;
    float currentSpeedRight_dps = 0.0f;
    float targetPitchOffset_deg = 0.0f;
    float targetAngularVelocity_dps = 0.0f;
};

