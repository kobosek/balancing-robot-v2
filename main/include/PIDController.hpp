#pragma once

#include "interfaces/IPIDController.hpp"
#include "esp_log.h"
#include <string>

class PIDController : public IPIDController {
public:
    PIDController(std::string config_key);

    esp_err_t init(const PIDConfig& config) override;
    esp_err_t onConfigUpdate(const PIDConfig& config) override;

    // Implement compute and reset
    float compute(float setpoint, float currentValue, float dt) override;
    void reset() override;

    // Implement IConfigObserver methods using the key
    esp_err_t init(const IRuntimeConfig& runtimeConfig) override;
    esp_err_t onConfigUpdate(const IRuntimeConfig& runtimeConfig) override;
private:
    static constexpr const char* TAG = "PIDController";
    
    float m_integral = 0.0f;
    float m_lastError = 0.0f;

    // Key to identify which config section applies to this instance
    std::string m_config_key;

    esp_err_t setParams(const PIDConfig& config);

    float m_kp, m_ki, m_kd;
    float m_outputMin, m_outputMax;
    float m_iTermMin, m_iTermMax;
};
