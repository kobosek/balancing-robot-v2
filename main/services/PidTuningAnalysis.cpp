#include "PidTuningAnalysis.hpp"

#include "esp_log.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr const char* TAG = "PidTuningAnalysis";
constexpr float MIN_IDENTIFICATION_TIME_S = 0.02f;
constexpr float MAX_RESPONSE_GAIN_RATIO = 2.5f;
constexpr float MAX_RESPONSE_TIME_RATIO = 4.0f;
constexpr float MAX_DIRECTION_GAIN_RATIO = 3.5f;

float averagePositive(float a, float b) {
    return (a + b) * 0.5f;
}
}

PidTuningStepResult PidTuningStepAnalyzer::analyzeStep(const std::vector<PidTuningStepSample>& samples,
                                                       float baseline_dps,
                                                       float effort,
                                                       const PidTuningConfig& config) {
    PidTuningStepResult result;
    if (samples.empty()) {
        return result;
    }

    const float duration_s = config.step_duration_ms / 1000.0f;
    const float finalWindowStart_s = std::max(duration_s * 0.75f, duration_s - 0.35f);
    float finalSum = 0.0f;
    int finalCount = 0;
    for (const auto& sample : samples) {
        if (sample.time_s >= finalWindowStart_s) {
            finalSum += sample.speed_dps;
            ++finalCount;
        }
    }
    if (finalCount == 0) {
        return result;
    }

    const float steadySpeed = finalSum / static_cast<float>(finalCount);
    float maxFinalDeviation = 0.0f;
    for (const auto& sample : samples) {
        if (sample.time_s >= finalWindowStart_s) {
            maxFinalDeviation = std::max(maxFinalDeviation, std::fabs(sample.speed_dps - steadySpeed));
        }
    }
    const float delta = steadySpeed - baseline_dps;
    const float absDelta = std::fabs(delta);
    if (absDelta < config.min_response_dps || delta * effort <= 0.0f) {
        return result;
    }

    const float maxAllowedDeviation = std::max(20.0f, absDelta * 0.35f);
    if (maxFinalDeviation > maxAllowedDeviation) {
        ESP_LOGW(TAG,
                 "Rejecting unstable step response: final deviation %.1f dps exceeds %.1f dps",
                 maxFinalDeviation,
                 maxAllowedDeviation);
        return result;
    }

    float t10 = duration_s;
    float t632 = duration_s;
    bool found10 = false;
    bool found632 = false;
    for (const auto& sample : samples) {
        const float response = std::fabs(sample.speed_dps - baseline_dps);
        if (!found10 && response >= absDelta * 0.10f) {
            t10 = sample.time_s;
            found10 = true;
        }
        if (!found632 && response >= absDelta * 0.632f) {
            t632 = sample.time_s;
            found632 = true;
        }
    }

    if (!found10 || !found632) {
        return result;
    }

    result.valid = true;
    result.gain_dps_per_effort = absDelta / std::fabs(effort);
    result.deadTime_s = std::max(t10, MIN_IDENTIFICATION_TIME_S);
    result.timeConstant_s = std::max(t632 - result.deadTime_s, MIN_IDENTIFICATION_TIME_S);
    result.steadySpeed_dps = steadySpeed;
    return result;
}

bool PidTuningStepAnalyzer::averageStepResults(const std::vector<PidTuningStepDefinition>& stepPlan,
                                               const std::vector<PidTuningStepResult>& stepResults,
                                               bool leftWheel,
                                               bool forward,
                                               PidTuningStepResult& averageResult,
                                               std::string& error) {
    float gainSum = 0.0f;
    float deadSum = 0.0f;
    float timeConstantSum = 0.0f;
    float steadySpeedSum = 0.0f;
    float minGain = 0.0f;
    float maxGain = 0.0f;
    float minDead = 0.0f;
    float maxDead = 0.0f;
    float minTimeConstant = 0.0f;
    float maxTimeConstant = 0.0f;
    int count = 0;

    for (size_t i = 0; i < stepPlan.size() && i < stepResults.size(); ++i) {
        const PidTuningStepDefinition& step = stepPlan[i];
        const PidTuningStepResult& result = stepResults[i];
        const bool stepForward = step.effort > 0.0f;
        if (step.leftWheel != leftWheel || stepForward != forward || !result.valid) {
            continue;
        }

        gainSum += result.gain_dps_per_effort;
        deadSum += result.deadTime_s;
        timeConstantSum += result.timeConstant_s;
        steadySpeedSum += result.steadySpeed_dps;
        if (count == 0) {
            minGain = maxGain = result.gain_dps_per_effort;
            minDead = maxDead = result.deadTime_s;
            minTimeConstant = maxTimeConstant = result.timeConstant_s;
        } else {
            minGain = std::min(minGain, result.gain_dps_per_effort);
            maxGain = std::max(maxGain, result.gain_dps_per_effort);
            minDead = std::min(minDead, result.deadTime_s);
            maxDead = std::max(maxDead, result.deadTime_s);
            minTimeConstant = std::min(minTimeConstant, result.timeConstant_s);
            maxTimeConstant = std::max(maxTimeConstant, result.timeConstant_s);
        }
        ++count;
    }

    if (count <= 0) {
        error = "Tuning aborted: missing motor response samples";
        return false;
    }

    if (count > 1) {
        if (minGain <= 0.0f || (maxGain / minGain) > MAX_RESPONSE_GAIN_RATIO) {
            error = "Tuning aborted: motor gain changed too much between effort levels";
            return false;
        }
        if (minDead <= 0.0f || (maxDead / minDead) > MAX_RESPONSE_TIME_RATIO ||
            minTimeConstant <= 0.0f || (maxTimeConstant / minTimeConstant) > MAX_RESPONSE_TIME_RATIO) {
            error = "Tuning aborted: motor timing changed too much between effort levels";
            return false;
        }
    }

    const float invCount = 1.0f / static_cast<float>(count);
    averageResult.valid = true;
    averageResult.gain_dps_per_effort = gainSum * invCount;
    averageResult.deadTime_s = deadSum * invCount;
    averageResult.timeConstant_s = timeConstantSum * invCount;
    averageResult.steadySpeed_dps = steadySpeedSum * invCount;
    return true;
}

bool PidTuningStepAnalyzer::analyzeValidation(float validationFinalSum_dps,
                                              int validationFinalCount,
                                              float target_dps,
                                              float minResponse_dps,
                                              const std::string& wheelName,
                                              std::string& error) {
    if (validationFinalCount <= 0) {
        error = "Tuning aborted: no validation samples";
        return false;
    }

    const float averageSpeed = validationFinalSum_dps / static_cast<float>(validationFinalCount);
    if (averageSpeed * target_dps <= 0.0f ||
        std::fabs(averageSpeed) < std::max(minResponse_dps, std::fabs(target_dps) * 0.25f)) {
        error = "Tuning aborted: " + wheelName + " validation response too weak";
        return false;
    }

    return true;
}

PidTuningCandidateResult PidTuningCandidateCalculator::compute(const std::vector<PidTuningStepDefinition>& stepPlan,
                                                               const std::vector<PidTuningStepResult>& stepResults,
                                                               bool tuneLeft,
                                                               const PIDConfig& originalConfig,
                                                               const PidTuningConfig& tuningConfig) {
    PidTuningCandidateResult output;
    for (const auto& result : stepResults) {
        if (!result.valid ||
            !std::isfinite(result.gain_dps_per_effort) ||
            !std::isfinite(result.deadTime_s) ||
            !std::isfinite(result.timeConstant_s)) {
            output.error = "Tuning aborted: incomplete step response";
            return output;
        }
    }

    PidTuningStepResult forward;
    PidTuningStepResult reverse;
    if (!PidTuningStepAnalyzer::averageStepResults(stepPlan, stepResults, tuneLeft, true, forward, output.error) ||
        !PidTuningStepAnalyzer::averageStepResults(stepPlan, stepResults, tuneLeft, false, reverse, output.error)) {
        if (output.error.empty()) {
            output.error = "Tuning aborted: missing averaged motor response";
        }
        return output;
    }

    const float minGain = std::min(forward.gain_dps_per_effort, reverse.gain_dps_per_effort);
    const float maxGain = std::max(forward.gain_dps_per_effort, reverse.gain_dps_per_effort);
    if (minGain <= 0.0f || (maxGain / minGain) > MAX_DIRECTION_GAIN_RATIO) {
        output.error = "Tuning aborted: forward/reverse motor gains are inconsistent";
        return output;
    }

    const float gain = averagePositive(forward.gain_dps_per_effort, reverse.gain_dps_per_effort);
    const float dead = std::max(averagePositive(forward.deadTime_s, reverse.deadTime_s), MIN_IDENTIFICATION_TIME_S);
    const float timeConstant = std::max(averagePositive(forward.timeConstant_s, reverse.timeConstant_s), MIN_IDENTIFICATION_TIME_S);
    const float kp = tuningConfig.gain_scale * 1.2f * timeConstant / (gain * dead);

    PIDConfig candidate = originalConfig;
    candidate.pid_kp = std::max(0.0f, std::min(500.0f, kp));
    candidate.pid_ki = std::max(0.0f, std::min(500.0f, candidate.pid_kp / (2.0f * dead)));
    candidate.pid_kd = std::max(0.0f, std::min(500.0f, candidate.pid_kp * 0.5f * dead));

    if (!std::isfinite(candidate.pid_kp) ||
        !std::isfinite(candidate.pid_ki) ||
        !std::isfinite(candidate.pid_kd) ||
        candidate.pid_kp <= 1e-6f) {
        output.error = "Tuning aborted: computed invalid PID gains";
        return output;
    }

    output.metrics.valid = true;
    output.metrics.forwardGain_dps_per_effort = forward.gain_dps_per_effort;
    output.metrics.reverseGain_dps_per_effort = reverse.gain_dps_per_effort;
    output.metrics.deadTime_s = dead;
    output.metrics.timeConstant_s = timeConstant;
    output.metrics.steadySpeedForward_dps = forward.steadySpeed_dps;
    output.metrics.steadySpeedReverse_dps = reverse.steadySpeed_dps;
    output.candidate = candidate;
    output.success = true;
    return output;
}
