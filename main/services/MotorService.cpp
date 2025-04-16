// main/MotorService.cpp
#include "MotorService.hpp"             // Relative path within module's include dir
#include "SystemStateChangedEvent.hpp"  // Found via INCLUDE_DIRS
#include "MX1616H_HWDriver.hpp"         // Found via INCLUDE_DIRS (needed for make_unique)
#include "BaseEvent.hpp"                // Found via INCLUDE_DIRS (needed for handle state change)
#include "EventTypes.hpp"               // Found via INCLUDE_DIRS (needed for handle state change)
#include "SystemState.hpp"              // Found via INCLUDE_DIRS (needed for handle state change)
#include "esp_check.h"
#include "driver/ledc.h"
#include <cmath>
#include <mutex> // <<< ADDED: Needed if accessing state concurrently
#include <algorithm>
#include "esp_log.h"                    // Moved from header
#include <memory>                        // Moved from header


MotorService::MotorService(const MotorConfig& config, EventBus& bus) :
    m_config(config),
    m_eventBus(bus),
    m_current_system_state(SystemState::INIT),
    m_enabled(false),
    m_pwm_max_duty(0)
{
    m_pwm_max_duty = (1 << m_config.duty_resolution) - 1;
    ESP_LOGI(TAG, "PWM Max Duty calculated: %lu for %d bits resolution", m_pwm_max_duty, m_config.duty_resolution);

    m_hw_driver_left = std::make_unique<MX1616H_HWDriver>(
        m_config.left_pin_in1, m_config.left_pin_in2,
        m_config.left_channel_1, m_config.left_channel_2,
        m_config.timer_num, m_config.speed_mode, m_config.duty_resolution, m_config.pwm_frequency_hz
    );
    m_hw_driver_right = std::make_unique<MX1616H_HWDriver>(
        m_config.right_pin_in1, m_config.right_pin_in2,
        m_config.right_channel_1, m_config.right_channel_2,
        m_config.timer_num, m_config.speed_mode, m_config.duty_resolution, m_config.pwm_frequency_hz
    );
}

esp_err_t MotorService::init() {
    ESP_LOGI(TAG, "Initializing MotorService...");

    esp_err_t ret = configureLEDCTimer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Aborting MotorService init due to timer config failure.");
        return ret;
    }
    ESP_LOGI(TAG, "LEDC Timer %d configuration verified/completed.", (int)m_config.timer_num);

    ESP_RETURN_ON_FALSE(m_hw_driver_left != nullptr, ESP_FAIL, TAG, "Left HW Driver is null pointer");
    ret = m_hw_driver_left->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize Left HW Driver");

    ESP_RETURN_ON_FALSE(m_hw_driver_right != nullptr, ESP_FAIL, TAG, "Right HW Driver is null pointer");
    ret = m_hw_driver_right->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize Right HW Driver");

    m_enabled = false;
    ret = setMotorEffort(0.0f, 0.0f);
    if(ret != ESP_OK) { ESP_LOGE(TAG, "Failed to set initial motor effort to zero during init!"); }

    ESP_LOGI(TAG, "MotorService Initialized Successfully.");
    return ESP_OK;
}

// <<< ADDED: Event subscription logic >>>
void MotorService::subscribeToEvents(EventBus& bus) {
    bus.subscribe(EventType::SYSTEM_STATE_CHANGED,
        [this](const BaseEvent& ev){
            // This lambda captures 'this' and calls the member handler
            this->handleSystemStateChange(ev);
        }
    );
    ESP_LOGI(TAG, "Subscribed to SYSTEM_STATE_CHANGED events.");
}

esp_err_t MotorService::configureLEDCTimer() {
     ESP_LOGI(TAG, "Configuring LEDC Timer %d: Freq=%luHz, Res=%d bits, Mode=%d",
              (int)m_config.timer_num, m_config.pwm_frequency_hz, m_config.duty_resolution, (int)m_config.speed_mode);

     if (m_config.speed_mode != LEDC_LOW_SPEED_MODE
#if SOC_LEDC_SUPPORT_HS_MODE
         && m_config.speed_mode != LEDC_HIGH_SPEED_MODE
#endif
     ) {
          ESP_LOGE(TAG, "Configuration error: Invalid speed mode (%d) requested in config.", (int)m_config.speed_mode);
          return ESP_ERR_INVALID_ARG;
     }

     ledc_timer_config_t ledc_timer = {
        .speed_mode       = m_config.speed_mode,
        .duty_resolution  = m_config.duty_resolution,
        .timer_num        = m_config.timer_num,
        .freq_hz          = m_config.pwm_frequency_hz,
        .clk_cfg          = LEDC_AUTO_CLK};

    esp_err_t ret = ledc_timer_config(&ledc_timer);

    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "LEDC timer %d already configured. Assuming compatible settings.", (int)m_config.timer_num);
        return ESP_OK;
    } else if (ret != ESP_OK) {
         ESP_LOGE(TAG, "ledc_timer_config failed for timer %d: %s (%d)", (int)m_config.timer_num, esp_err_to_name(ret), ret);
         return ret;
    }

    ESP_LOGI(TAG, "LEDC timer %d configured successfully for speed mode %d.", (int)m_config.timer_num, (int)m_config.speed_mode);
    return ESP_OK;
}


esp_err_t MotorService::setMotorEffort(float leftEffort, float rightEffort) {
    // (Implementation remains the same)
    leftEffort = std::max(-1.0f, std::min(1.0f, leftEffort));
    rightEffort = std::max(-1.0f, std::min(1.0f, rightEffort));
    uint32_t leftDuty1 = 0, leftDuty2 = 0, rightDuty1 = 0, rightDuty2 = 0;
    if (m_enabled) {
        if (std::fabs(leftEffort) > 1e-3) {
            float mag = std::fabs(leftEffort);
            uint32_t effective_max_duty = m_pwm_max_duty;
            uint32_t effective_deadzone = std::min(m_config.deadzone_duty, effective_max_duty);
            uint32_t duty_range = (effective_max_duty > effective_deadzone) ? (effective_max_duty - effective_deadzone) : 0;
            uint32_t dutyMag = effective_deadzone + static_cast<uint32_t>(mag * duty_range);
            dutyMag = std::min(dutyMag, effective_max_duty);
            if (leftEffort > 0) { leftDuty1 = dutyMag; } else { leftDuty2 = dutyMag; }
        }
        if (std::fabs(rightEffort) > 1e-3) {
            float mag = std::fabs(rightEffort);
             uint32_t effective_max_duty = m_pwm_max_duty;
             uint32_t effective_deadzone = std::min(m_config.deadzone_duty, effective_max_duty);
             uint32_t duty_range = (effective_max_duty > effective_deadzone) ? (effective_max_duty - effective_deadzone) : 0;
             uint32_t dutyMag = effective_deadzone + static_cast<uint32_t>(mag * duty_range);
             dutyMag = std::min(dutyMag, effective_max_duty);
             if (rightEffort > 0) { rightDuty1 = dutyMag; } else { rightDuty2 = dutyMag; }
        }
    }
    ESP_LOGV(TAG, "Set Effort: L=%.2f R=%.2f => Raw Duty L(%lu,%lu) R(%lu,%lu) | Enabled:%d", leftEffort, rightEffort, leftDuty1, leftDuty2, rightDuty1, rightDuty2, m_enabled);
    esp_err_t ret_l = ESP_FAIL, ret_r = ESP_FAIL;
    if (m_hw_driver_left) { ret_l = m_hw_driver_left->setRawDuty(leftDuty1, leftDuty2); if (ret_l != ESP_OK) ESP_LOGE(TAG, "Failed set left motor duty: %s", esp_err_to_name(ret_l)); } else { ESP_LOGE(TAG, "Left HW Driver null!"); }
    if (m_hw_driver_right) { ret_r = m_hw_driver_right->setRawDuty(rightDuty1, rightDuty2); if (ret_r != ESP_OK) ESP_LOGE(TAG, "Failed set right motor duty: %s", esp_err_to_name(ret_r)); } else { ESP_LOGE(TAG, "Right HW Driver null!"); }
    return (ret_l == ESP_OK && ret_r == ESP_OK) ? ESP_OK : ESP_FAIL;
}

void MotorService::handleSystemStateChange(const BaseEvent& event) {
    // (Implementation remains the same)
    if(event.type != EventType::SYSTEM_STATE_CHANGED) return;
    const auto& stateEvent = static_cast<const SystemStateChangedEvent&>(event);
    m_current_system_state = stateEvent.newState;
    ESP_LOGI(TAG, "Received State Change event: %d", static_cast<int>(m_current_system_state));
    bool should_be_enabled = (m_current_system_state == SystemState::BALANCING);
    if (should_be_enabled && !m_enabled) { ESP_LOGI(TAG, "Enabling motors."); m_enabled = true; }
    else if (!should_be_enabled && m_enabled) { ESP_LOGI(TAG, "Disabling motors."); m_enabled = false; esp_err_t stop_ret = setMotorEffort(0.0f, 0.0f); if(stop_ret != ESP_OK) { ESP_LOGE(TAG, "Failed stop motors!"); } }
}