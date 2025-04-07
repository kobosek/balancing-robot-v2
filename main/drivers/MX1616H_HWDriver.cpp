// main/MX1616H_HWDriver.cpp
#include "MX1616H_HWDriver.hpp"         // Relative path within module's include dir
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

MX1616H_HWDriver::MX1616H_HWDriver(gpio_num_t pin_in1, gpio_num_t pin_in2,
                                 ledc_channel_t channel1, ledc_channel_t channel2,
                                 ledc_timer_t timer_num, ledc_mode_t speed_mode,
                                 ledc_timer_bit_t duty_resolution, uint32_t pwm_frequency) :
    m_pin_in1(pin_in1),
    m_pin_in2(pin_in2),
    m_channel1(channel1),
    m_channel2(channel2),
    m_timer_num(timer_num),
    m_speed_mode(speed_mode),
    m_duty_resolution(duty_resolution),
    m_pwm_frequency(pwm_frequency),
    m_is_initialized(false)
{}


esp_err_t MX1616H_HWDriver::init() {
    if (m_is_initialized) {
        ESP_LOGW(TAG, "HW driver already initialized.");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Initializing HW driver for IN1:%d(CH%d), IN2:%d(CH%d) using Timer%d",
             (int)m_pin_in1, (int)m_channel1, (int)m_pin_in2, (int)m_channel2, (int)m_timer_num);

    // Timer configuration is assumed to be done externally by MotorService

    // Configure LEDC Channels (without speed_mode field)
     ledc_channel_config_t ledc_channel_conf[2] = {
        {
            .gpio_num       = m_pin_in1,
            .channel        = m_channel1,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = m_timer_num,
            .duty           = 0,
            .hpoint         = 0,
            .flags          = { .output_invert = 0 }
        },
        {
            .gpio_num       = m_pin_in2,
            .channel        = m_channel2,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = m_timer_num,
            .duty           = 0,
            .hpoint         = 0,
            .flags          = { .output_invert = 0 }
        }
    };

    esp_err_t ret = ledc_channel_config(&ledc_channel_conf[0]);
    if (ret != ESP_OK) {
         ESP_LOGE(TAG, "ledc_channel_config failed for CH%d (GPIO %d) on Timer%d: %s (%d)",
                  (int)m_channel1, (int)m_pin_in1, (int)m_timer_num, esp_err_to_name(ret), ret);
        return ret;
    }
    ESP_LOGD(TAG, "Channel %d (GPIO %d) configured.", m_channel1, m_pin_in1);

    ret = ledc_channel_config(&ledc_channel_conf[1]);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "ledc_channel_config failed for CH%d (GPIO %d) on Timer%d: %s (%d)",
                  (int)m_channel2, (int)m_pin_in2, (int)m_timer_num, esp_err_to_name(ret), ret);
        return ret;
    }
    ESP_LOGD(TAG, "Channel %d (GPIO %d) configured.", m_channel2, m_pin_in2);

    m_is_initialized = true; // Set flag *before* calling setRawDuty

    ret = setRawDuty(0, 0); // Ensure motors are stopped initially
    if (ret != ESP_OK) {
        m_is_initialized = false; // Reset flag on error
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set initial duty to 0");
    }

    ESP_LOGI(TAG, "HW driver initialized.");
    return ESP_OK;
}


esp_err_t MX1616H_HWDriver::setRawDuty(uint32_t duty1, uint32_t duty2) {
    if (!m_is_initialized) {
        ESP_LOGE(TAG, "Driver not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;
    ESP_LOGV(TAG, "Setting Raw Duty CH%d=%lu, CH%d=%lu", m_channel1, duty1, m_channel2, duty2);

    // Use the speed mode associated with the timer for API calls
    ledc_mode_t current_speed_mode = m_speed_mode;

    ret = ledc_set_duty(current_speed_mode, m_channel1, duty1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed set duty chan %d", m_channel1);
    ret = ledc_update_duty(current_speed_mode, m_channel1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed update duty chan %d", m_channel1);

    ret = ledc_set_duty(current_speed_mode, m_channel2, duty2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed set duty chan %d", m_channel2);
    ret = ledc_update_duty(current_speed_mode, m_channel2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed update duty chan %d", m_channel2);

    return ESP_OK;
}