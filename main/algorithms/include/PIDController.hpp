#pragma once

#include "control_math/PidCore.hpp"
#include "config/PIDConfig.hpp"
#include <string>
#include "esp_err.h"

class PIDController {
public:
    PIDController(std::string config_key);
    ~PIDController() = default; // Good practice to add destructor declaration

    esp_err_t init(const PIDConfig& config);
    esp_err_t updateParams(const PIDConfig& config);
    float compute(float setpoint, float currentValue, float dt);
    float computeWithMeasurementRate(float setpoint, float currentValue, float currentRate, float dt);
    void reset();

private:
    static constexpr const char* TAG = "PIDController";

    // The application-facing adapter retains the existing API and config
    // type; all PID state and arithmetic live in the portable core.
    std::string m_config_key;
    PIDConfig m_params;
    control_math::PidCore m_core;
};
