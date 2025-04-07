// main/include/MX1616H_HWDriver.hpp
#pragma once
      
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include <stdint.h>


class MX1616H_HWDriver {
public:
     MX1616H_HWDriver(gpio_num_t pin_in1, gpio_num_t pin_in2,
                      ledc_channel_t channel1, ledc_channel_t channel2,
                      ledc_timer_t timer_num, ledc_mode_t speed_mode,
                      ledc_timer_bit_t duty_resolution, uint32_t pwm_frequency);

    // Declarations only
    esp_err_t init();
    esp_err_t setRawDuty(uint32_t duty1, uint32_t duty2);

private:
    static constexpr const char* TAG = "MX1616H_HW";
    const gpio_num_t m_pin_in1;
    const gpio_num_t m_pin_in2;
    const ledc_channel_t m_channel1;
    const ledc_channel_t m_channel2;
    const ledc_timer_t m_timer_num;
    const ledc_mode_t m_speed_mode;
    const ledc_timer_bit_t m_duty_resolution;
    const uint32_t m_pwm_frequency;
    bool m_is_initialized = false;
};