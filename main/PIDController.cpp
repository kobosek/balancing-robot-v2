#include "include/PIDController.hpp"
#include "include/RuntimeConfig.hpp"
#include <algorithm>

PIDController::PIDController(std::string config_key) : m_config_key(std::move(config_key)) {
    ESP_LOGI(TAG, "PIDController instance created for key: %s", m_config_key.c_str());
}
esp_err_t PIDController::setParams(const PIDConfig& config) {
    ESP_LOGD(TAG, "Setting PID parameters for key: %s", m_config_key.c_str());
    m_kp = config.pid_kp;
    m_ki = config.pid_ki;
    m_kd = config.pid_kd;
    m_outputMin = config.pid_output_min;
    m_outputMax = config.pid_output_max;
    m_iTermMin = config.pid_iterm_min;
    m_iTermMax = config.pid_iterm_max;

    ESP_LOGD(TAG, "PID parameters set - Kp: %.4f, Ki: %.4f, Kd: %.4f, OutMin: %.2f, OutMax: %.2f, ITermMin: %.2f, ITermMax: %.2f",
             m_kp, m_ki, m_kd, m_outputMin, m_outputMax, m_iTermMin, m_iTermMax);

    reset(); // Reset state when parameters change
    return ESP_OK;
}

esp_err_t PIDController::init(const IRuntimeConfig& runtimeConfig) {
    ESP_LOGI(TAG, "Initializing PID Controller for key: %s", m_config_key.c_str());
    return onConfigUpdate(runtimeConfig); // Apply initial config
}

esp_err_t PIDController::onConfigUpdate(const IRuntimeConfig& runtimeConfig) {
    ESP_LOGI(TAG, "Updating PID Controller configuration for key: %s", m_config_key.c_str());

    PIDConfig specific_config;
    bool config_found = false;

    // --- Modified Start: Use if/else if and flag, return error code ---
    if (m_config_key == "angle") {
        specific_config = runtimeConfig.getAnglePidConfig();
        config_found = true;
    } else if (m_config_key == "speed_left") {
        specific_config = runtimeConfig.getSpeedPidLeftConfig();
        config_found = true;
    } else if (m_config_key == "speed_right") {
        specific_config = runtimeConfig.getSpeedPidRightConfig();
        config_found = true;
    }

    if (config_found) {
        // Apply the found configuration
        return setParams(specific_config);
    } else {
        // Log error and return an appropriate error code
        ESP_LOGE(TAG, "Configuration key '%s' not recognized. Cannot apply update.", m_config_key.c_str());
        return ESP_ERR_NOT_FOUND; // Or ESP_FAIL if preferred
    }
    // --- Modified End ---
}

esp_err_t PIDController::init(const PIDConfig& config) {
    ESP_LOGI(TAG, "Directly initializing PID Controller for key: %s with provided config", m_config_key.c_str());
    return setParams(config);
}
 esp_err_t PIDController::onConfigUpdate(const PIDConfig& config) {
    ESP_LOGI(TAG, "Directly updating PID Controller for key: %s with provided config", m_config_key.c_str());
    return setParams(config);
}

float PIDController::compute(float setpoint, float currentValue, float dt) {
    // Check for valid dt
    if (dt <= 0.0f) {
         ESP_LOGW(TAG, "Invalid dt (%.4f) in PID compute for key %s, returning 0 output.", dt, m_config_key.c_str());
         // Optionally return last output or 0
         return 0.0f; // Or handle appropriately
    }

    float currentError = setpoint - currentValue;

    // Proportional term
    float pTerm = m_kp * currentError;

    // Integral term - Accumulate using trapezoidal rule for better accuracy
    m_integral += (currentError + m_lastError) / 2.0f * dt;
    // Apply anti-windup (clamping)
    m_integral = std::max(m_iTermMin, std::min(m_integral, m_iTermMax));

    float iTerm = m_ki * m_integral;

    // Derivative term - Avoid division by zero if dt is extremely small
    float dTerm = 0.0f;
    if (dt > 1e-6) { // Avoid division by zero or near-zero dt
         dTerm = m_kd * (currentError - m_lastError) / dt;
    }

    // Update last error *after* calculating derivative term
    m_lastError = currentError;

    // Calculate total output
    float output = pTerm + iTerm + dTerm;

    // Clamp output
    output = std::max(m_outputMin, std::min(output, m_outputMax));

    ESP_LOGV(TAG, "PID Computation (%s) - Set: %.2f, Curr: %.2f, Err: %.2f, P: %.2f, I: %.2f (Acc: %.2f), D: %.2f, Out: %.2f",
                    m_config_key.c_str(), setpoint, currentValue, currentError, pTerm, iTerm, m_integral, dTerm, output);

    return output;
}

void PIDController::reset() {
    ESP_LOGD(TAG, "Resetting PID state for key: %s", m_config_key.c_str());
    m_integral = 0.0f;
    m_lastError = 0.0f;
}



