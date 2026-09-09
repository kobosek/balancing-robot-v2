// main/PIDController.cpp
#include "PIDController.hpp"
#include <utility>
#include "esp_log.h"
#include "esp_err.h"


PIDController::PIDController(std::string config_key) :
    m_config_key(std::move(config_key)),
    m_params(),
    m_core()
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

    m_core.setParameters({
        config.pid_kp,
        config.pid_ki,
        config.pid_kd,
        config.pid_output_min,
        config.pid_output_max,
        config.pid_iterm_min,
        config.pid_iterm_max
    });

    ESP_LOGD(TAG, "PID parameters set - Kp: %.4f, Ki: %.4f, Kd: %.4f, OutMin: %.2f, OutMax: %.2f, ITermMin: %.2f, ITermMax: %.2f",
             m_params.pid_kp, m_params.pid_ki, m_params.pid_kd, m_params.pid_output_min, m_params.pid_output_max, m_params.pid_iterm_min, m_params.pid_iterm_max);

    return ESP_OK;
}


float PIDController::compute(float setpoint, float currentValue, float dt) {
    const auto result = m_core.compute(setpoint, currentValue, dt);
    if (!result.valid) {
        ESP_LOGW(TAG, "Invalid PID input (dt %.4f) for key %s, returning 0 output.", dt, m_config_key.c_str());
        return 0.0f;
    }

    ESP_LOGV(TAG, "PID (%s) | SP:%.3f PV:%.3f E:%.3f | P:%.3f I:%.3f D:%.3f | Out:%.3f",
                    m_config_key.c_str(), setpoint, currentValue, setpoint - currentValue,
                    result.pTerm, result.integralTerm, result.derivativeTerm, result.output);

    return result.output;
}

float PIDController::computeWithMeasurementRate(float setpoint, float currentValue, float currentRate, float dt) {
    const auto result = m_core.computeWithMeasurementRate(setpoint, currentValue, currentRate, dt);
    if (!result.valid) {
        ESP_LOGW(TAG, "Invalid PID input (dt %.4f) for key %s, returning 0 output.", dt, m_config_key.c_str());
        return 0.0f;
    }

    ESP_LOGV(TAG, "PID (%s) | SP:%.3f PV:%.3f Rate:%.3f E:%.3f | P:%.3f I:%.3f D:%.3f | Out:%.3f",
                    m_config_key.c_str(), setpoint, currentValue, currentRate,
                    setpoint - currentValue, result.pTerm,
                    result.integralTerm, result.derivativeTerm, result.output);

    return result.output;
}

void PIDController::reset() {
    ESP_LOGD(TAG, "Resetting PID state for key: %s", m_config_key.c_str());
    m_core.reset();
}
