// main/PIDController.cpp
#include "PIDController.hpp"            // Relative path within module's include dir
#include <algorithm>                    // For std::max, std::min
#include <cmath>                        // For std::fabs
#include "esp_log.h"                    // Moved from header
#include "esp_err.h"


// Initializer list now matches declaration order in the updated header
PIDController::PIDController(std::string config_key) :
    m_config_key(std::move(config_key)), // 1st
    m_integral(0.0f),                    // 2nd
    m_lastError(0.0f),                   // 3rd
    m_params()                           // 4th (Default initialized)
{
    ESP_LOGD(TAG, "PIDController instance created for key: %s", m_config_key.c_str());
}

// Initialize PID controller with specific config struct
esp_err_t PIDController::init(const PIDConfig& config) {
    ESP_LOGI(TAG, "Initializing PID Controller for key: %s", m_config_key.c_str());
    return updateParams(config);
}

// Update PID parameters from a config struct
esp_err_t PIDController::updateParams(const PIDConfig& config) {
     ESP_LOGD(TAG, "Updating PID parameters for key: %s", m_config_key.c_str());
     m_params = config; // Store the whole struct

    ESP_LOGD(TAG, "PID parameters set - Kp: %.4f, Ki: %.4f, Kd: %.4f, OutMin: %.2f, OutMax: %.2f, ITermMin: %.2f, ITermMax: %.2f",
             m_params.pid_kp, m_params.pid_ki, m_params.pid_kd, m_params.pid_output_min, m_params.pid_output_max, m_params.pid_iterm_min, m_params.pid_iterm_max);

    reset(); // Reset state when parameters change
    return ESP_OK;
}


float PIDController::compute(float setpoint, float currentValue, float dt) {
    if (dt <= 0.0f) {
         ESP_LOGW(TAG, "Invalid dt (%.4f) in PID compute for key %s, returning 0 output.", dt, m_config_key.c_str());
         // Consider returning last valid output? Needs member variable.
         return 0.0f;
    }

    float currentError = setpoint - currentValue;
    float pTerm = m_params.pid_kp * currentError;

    // Integral term - Apply Ki during accumulation
    float integral_delta = (currentError + m_lastError) * 0.5f * dt * m_params.pid_ki;
    m_integral += integral_delta;

    // Anti-windup: Clamp the integral term itself
    m_integral = std::max(m_params.pid_iterm_min, std::min(m_integral, m_params.pid_iterm_max));

    // Derivative term
    float dTerm = 0.0f;
    if (dt > 1e-6) { // Avoid division by zero
         // dError/dt
         dTerm = m_params.pid_kd * (currentError - m_lastError) / dt;
         // Optional: Derivative on measurement instead (less prone to setpoint changes)
         // dTerm = -m_params.pid_kd * (currentValue - last_measurement) / dt; // Need last_measurement state
    }
    m_lastError = currentError; // Update last error *after* using it for I and D terms

    // Calculate total output
    float output = pTerm + m_integral + dTerm; // I-term already includes Ki

    // Clamp final output
    output = std::max(m_params.pid_output_min, std::min(output, m_params.pid_output_max));

    ESP_LOGV(TAG, "PID (%s) | SP:%.3f PV:%.3f E:%.3f | P:%.3f I:%.3f D:%.3f | Out:%.3f",
                    m_config_key.c_str(), setpoint, currentValue, currentError, pTerm, m_integral, dTerm, output);

    return output;
}

float PIDController::computeWithMeasurementRate(float setpoint, float currentValue, float currentRate, float dt) {
    if (dt <= 0.0f) {
         ESP_LOGW(TAG, "Invalid dt (%.4f) in PID compute for key %s, returning 0 output.", dt, m_config_key.c_str());
         return 0.0f;
    }

    float currentError = setpoint - currentValue;
    float pTerm = m_params.pid_kp * currentError;

    float integral_delta = (currentError + m_lastError) * 0.5f * dt * m_params.pid_ki;
    m_integral += integral_delta;
    m_integral = std::max(m_params.pid_iterm_min, std::min(m_integral, m_params.pid_iterm_max));

    // Derivative-on-measurement avoids a kick when the command changes target angle.
    float dTerm = -m_params.pid_kd * currentRate;
    m_lastError = currentError;

    float output = pTerm + m_integral + dTerm;
    output = std::max(m_params.pid_output_min, std::min(output, m_params.pid_output_max));

    ESP_LOGV(TAG, "PID (%s) | SP:%.3f PV:%.3f Rate:%.3f E:%.3f | P:%.3f I:%.3f D:%.3f | Out:%.3f",
                    m_config_key.c_str(), setpoint, currentValue, currentRate, currentError, pTerm, m_integral, dTerm, output);

    return output;
}

void PIDController::reset() {
    ESP_LOGD(TAG, "Resetting PID state for key: %s", m_config_key.c_str());
    m_integral = 0.0f;
    m_lastError = 0.0f;
    // Note: Parameters (m_params) are *not* reset
}
