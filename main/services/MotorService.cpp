// main/MotorService.cpp
#include "MotorService.hpp"             // Relative path within module's include dir
#include "MOTOR_OutputEnabledChanged.hpp"
#include "MX1616H_HWDriver.hpp"         // Found via INCLUDE_DIRS (needed for make_unique)
#include "BaseEvent.hpp"                // Found via INCLUDE_DIRS (needed for handle state change)
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include <cmath>
#include <algorithm>
#include "esp_log.h"                    // Moved from header
#include <memory>                        // Moved from header


MotorService::MotorService(const MotorConfig& config, EventBus& bus) :
    m_config(config),
    m_eventBus(bus),
    m_enabled(false),
    m_pwm_max_duty(0)
{
    m_pwm_max_duty = (1 << m_config.duty_resolution) - 1;
    ESP_LOGI(TAG, "PWM Max Duty calculated: %lu for %d bits resolution", m_pwm_max_duty, m_config.duty_resolution);

    m_hw_driver_left = std::make_unique<MX1616H_HWDriver>(
        static_cast<gpio_num_t>(m_config.left_pin_in1),
        static_cast<gpio_num_t>(m_config.left_pin_in2),
        static_cast<ledc_channel_t>(m_config.left_channel_1),
        static_cast<ledc_channel_t>(m_config.left_channel_2),
        static_cast<ledc_timer_t>(m_config.timer_num),
        static_cast<ledc_mode_t>(m_config.speed_mode),
        static_cast<ledc_timer_bit_t>(m_config.duty_resolution),
        m_config.pwm_frequency_hz
    );
    m_hw_driver_right = std::make_unique<MX1616H_HWDriver>(
        static_cast<gpio_num_t>(m_config.right_pin_in1),
        static_cast<gpio_num_t>(m_config.right_pin_in2),
        static_cast<ledc_channel_t>(m_config.right_channel_1),
        static_cast<ledc_channel_t>(m_config.right_channel_2),
        static_cast<ledc_timer_t>(m_config.timer_num),
        static_cast<ledc_mode_t>(m_config.speed_mode),
        static_cast<ledc_timer_bit_t>(m_config.duty_resolution),
        m_config.pwm_frequency_hz
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

// EventHandler implementation
void MotorService::handleEvent(const BaseEvent& event) {
    if (event.is<MOTOR_OutputEnabledChanged>()) {
        handleMotorOutputEnabledChanged(event.as<MOTOR_OutputEnabledChanged>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
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
        .speed_mode       = static_cast<ledc_mode_t>(m_config.speed_mode),
        .duty_resolution  = static_cast<ledc_timer_bit_t>(m_config.duty_resolution),
        .timer_num        = static_cast<ledc_timer_t>(m_config.timer_num),
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

esp_err_t MotorService::setMotorEffort(float leftEffort, float rightEffort, uint64_t armId, uint32_t generation,
                                        int64_t sampleTimestampUs, int64_t maxAgeUs) {
    const bool finite = std::isfinite(leftEffort) && std::isfinite(rightEffort);
    if (!finite) {
        leftEffort = rightEffort = 0.0f;
    }
    leftEffort = std::max(-1.0f, std::min(1.0f, leftEffort));
    rightEffort = std::max(-1.0f, std::min(1.0f, rightEffort));
    uint32_t leftDuty1 = 0, leftDuty2 = 0, rightDuty1 = 0, rightDuty2 = 0;
    {
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
    esp_err_t result;
    bool expired = false;
    {
        std::lock_guard<std::mutex> lock(m_outputMutex);
        const auto now = esp_timer_get_time();
        expired = maxAgeUs > 0 && (sampleTimestampUs <= 0 || now < sampleTimestampUs || now - sampleTimestampUs > maxAgeUs);
        if (expired) { m_revokedArm = std::max(m_revokedArm, armId); if (m_armId <= m_revokedArm) m_enabled = false; }
        if (!finite) { m_revokedArm = std::max(m_revokedArm, m_armId); m_enabled = false; }
        if (!m_enabled || armId != m_armId || generation != m_generation || armId <= m_revokedArm) {
            leftDuty1 = leftDuty2 = rightDuty1 = rightDuty2 = 0;
        }
        result = writeDutyLocked(leftDuty1, leftDuty2, rightDuty1, rightDuty2);
    }
    return result != ESP_OK ? result : (expired ? ESP_ERR_TIMEOUT : (finite ? ESP_OK : ESP_ERR_INVALID_ARG));
}

esp_err_t MotorService::writeDutyLocked(uint32_t leftDuty1, uint32_t leftDuty2,
                                        uint32_t rightDuty1, uint32_t rightDuty2) {
    // Always attempt both sides, including when one driver reports an error.
    const esp_err_t leftResult = m_hw_driver_left
        ? m_hw_driver_left->setRawDuty(leftDuty1, leftDuty2) : ESP_ERR_INVALID_STATE;
    const esp_err_t rightResult = m_hw_driver_right
        ? m_hw_driver_right->setRawDuty(rightDuty1, rightDuty2) : ESP_ERR_INVALID_STATE;
    return leftResult != ESP_OK ? leftResult : rightResult;
}

void MotorService::handleMotorOutputEnabledChanged(const MOTOR_OutputEnabledChanged& event) {
    esp_err_t stopResult = ESP_OK;
    {
        std::lock_guard<std::mutex> lock(m_outputMutex);
        if (event.armId < m_armId) return;
        m_armId = event.armId;
        m_generation = event.generation;
        if (!event.enabled) m_revokedArm = std::max(m_revokedArm, event.armId);
        m_enabled = event.enabled && event.armId > m_revokedArm;
        if (!m_enabled) {
            // Repeat the zero write even for an already-disabled state, so a
            // previous failed stop can be retried. Do not recursively lock.
            stopResult = writeDutyLocked(0, 0, 0, 0);
        }
    }
    if (stopResult != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop motors: %s", esp_err_to_name(stopResult));
    }
}

void MotorService::inhibitImu(uint64_t armId) {
    std::lock_guard<std::mutex> lock(m_outputMutex);
    m_revokedArm = std::max(m_revokedArm, armId);
    if (m_armId <= m_revokedArm) {
        m_enabled = false;
        (void)writeDutyLocked(0, 0, 0, 0);
    }
}
bool MotorService::isArmAllowed(uint64_t armId, uint32_t generation) {
    std::lock_guard<std::mutex> lock(m_outputMutex);
    return m_enabled && armId == m_armId && generation == m_generation && armId > m_revokedArm;
}
