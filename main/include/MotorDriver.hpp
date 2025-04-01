#pragma once

#include "interfaces/IMotorDriver.hpp"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

class L298N : public IMotorDriver {
    public:
        L298N(gpio_num_t in1_pin, gpio_num_t in2_pin, gpio_num_t pwm_pin, ledc_channel_t channel) 
            : IN1_PIN(in1_pin), IN2_PIN(in2_pin), PWM_PIN(pwm_pin), CHANNEL_NUM(channel) {};
        esp_err_t init(const IRuntimeConfig&) override;
        esp_err_t onConfigUpdate(const IRuntimeConfig&) override;
        esp_err_t setSpeed(float speed) const override;
    private:
        static constexpr const char* TAG = "L298N";

        gpio_num_t IN1_PIN;
        gpio_num_t IN2_PIN;
        gpio_num_t PWM_PIN;
        ledc_channel_t CHANNEL_NUM;
};

class MX1616H : public IMotorDriver {
    public:
        MX1616H(gpio_num_t in1_pin, gpio_num_t in2_pin, ledc_channel_t channel1, ledc_channel_t channel2) 
            : IN1_PIN(in1_pin), IN2_PIN(in2_pin), CHANNEL_NUM_1(channel1), CHANNEL_NUM_2(channel2) {};
        esp_err_t init(const IRuntimeConfig&) override;
        esp_err_t onConfigUpdate(const IRuntimeConfig&) override;
        esp_err_t setSpeed(float speed) const override; 
    private:
        static constexpr const char* TAG = "MX1616H";
        
        gpio_num_t IN1_PIN;
        gpio_num_t IN2_PIN;

        ledc_channel_t CHANNEL_NUM_1;
        ledc_channel_t CHANNEL_NUM_2;
    };