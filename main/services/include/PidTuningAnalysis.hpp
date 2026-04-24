#pragma once

#include "PidTuningTypes.hpp"
#include "config/PIDConfig.hpp"
#include "config/PidTuningConfig.hpp"
#include <string>
#include <vector>

struct PidTuningResponseMetrics {
    bool valid = false;
    float forwardGain_dps_per_effort = 0.0f;
    float reverseGain_dps_per_effort = 0.0f;
    float deadTime_s = 0.0f;
    float timeConstant_s = 0.0f;
    float steadySpeedForward_dps = 0.0f;
    float steadySpeedReverse_dps = 0.0f;
};

struct PidTuningStepSample {
    float time_s = 0.0f;
    float speed_dps = 0.0f;
};

struct PidTuningStepResult {
    bool valid = false;
    float gain_dps_per_effort = 0.0f;
    float deadTime_s = 0.0f;
    float timeConstant_s = 0.0f;
    float steadySpeed_dps = 0.0f;
};

struct PidTuningStepDefinition {
    bool leftWheel = true;
    float effort = 0.0f;
    PidTuningPhase phase = PidTuningPhase::IDLE;
};

struct PidTuningCandidateResult {
    bool success = false;
    PIDConfig candidate;
    PidTuningResponseMetrics metrics;
    std::string error;
};

class PidTuningStepAnalyzer {
public:
    static PidTuningStepResult analyzeStep(const std::vector<PidTuningStepSample>& samples,
                                           float baseline_dps,
                                           float effort,
                                           const PidTuningConfig& config);

    static bool averageStepResults(const std::vector<PidTuningStepDefinition>& stepPlan,
                                   const std::vector<PidTuningStepResult>& stepResults,
                                   bool leftWheel,
                                   bool forward,
                                   PidTuningStepResult& averageResult,
                                   std::string& error);

    static bool analyzeValidation(float validationFinalSum_dps,
                                  int validationFinalCount,
                                  float target_dps,
                                  float minResponse_dps,
                                  const std::string& wheelName,
                                  std::string& error);
};

class PidTuningCandidateCalculator {
public:
    static PidTuningCandidateResult compute(const std::vector<PidTuningStepDefinition>& stepPlan,
                                            const std::vector<PidTuningStepResult>& stepResults,
                                            bool tuneLeft,
                                            const PIDConfig& originalConfig,
                                            const PidTuningConfig& tuningConfig);
};
