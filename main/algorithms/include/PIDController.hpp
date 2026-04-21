#pragma once

#include "ConfigData.hpp"               // Found via INCLUDE_DIRS (Need PIDConfig struct definition)
#include <string>
#include "esp_err.h"

class PIDController {
public:
    PIDController(std::string config_key);
    ~PIDController() = default; // Good practice to add destructor declaration

    esp_err_t init(const PIDConfig& config);
    esp_err_t updateParams(const PIDConfig& config);
    float compute(float setpoint, float currentValue, float dt);
    void reset();

private:
    static constexpr const char* TAG = "PIDController";

    // Declaration Order (Matching Initializer List in .cpp)
    std::string m_config_key; // 1st
    float m_integral;         // 2nd
    float m_lastError;        // 3rd
    PIDConfig m_params;       // 4th
};