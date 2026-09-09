#pragma once

#include <cmath>

// A small, allocation-free profile used by the longitudinal strategy.  It
// only shapes the requested linear velocity; feedback and safety decisions
// remain with the strategy and RobotController.
struct LongitudinalMotionProfileConfig {
    float maxVelocityMps = 0.0f;
    float maxAccelerationMps2 = 0.0f;
    float maxDecelerationMps2 = 0.0f;
};

struct LongitudinalMotionProfileResult {
    float targetVelocityMps = 0.0f;
    bool valid = false;
    bool limited = false;
    bool atRest = true;
    bool reachedCommand = false;
};

class LongitudinalMotionProfile {
public:
    explicit LongitudinalMotionProfile(const LongitudinalMotionProfileConfig& config = {});

    void configure(const LongitudinalMotionProfileConfig& config);
    void reset(float velocityMps = 0.0f);

    LongitudinalMotionProfileResult update(float commandVelocityMps,
                                           bool commandActive,
                                           float dtSeconds);

    float targetVelocityMps() const { return m_targetVelocityMps; }

private:
    LongitudinalMotionProfileConfig m_config;
    float m_targetVelocityMps = 0.0f;

    float clampCommand(float commandVelocityMps) const;
    static float advanceToward(float current, float target, float rate,
                               float dtSeconds, bool& limited);
};

