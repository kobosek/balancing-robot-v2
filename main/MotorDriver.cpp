#include "include/MotorDriver.hpp"
#include <math.h>

esp_err_t L298N::init(const IRuntimeConfig& config) {
    ESP_LOGI(TAG, "Initializing L298N motor driver");

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<IN1_PIN) | (1ULL<<IN2_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO pins");
        return ret;
    }

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer");
        return ret;
    }

    ledc_channel_config_t ledc_channel = {
            .gpio_num       = PWM_PIN,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = CHANNEL_NUM,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0,
            .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel %d", CHANNEL_NUM);
        return ret;
    }

    ESP_LOGI(TAG, "L298N motor driver initialized successfully");
    return ESP_OK;
}

esp_err_t L298N::setSpeed(float speed) const {
    ESP_LOGD(TAG, "Setting L298N speed to %.2f", speed);

    if (speed < 0) {
        gpio_set_level(IN1_PIN, 1);
        gpio_set_level(IN2_PIN, 0);
    } else {
        gpio_set_level(IN1_PIN, 0);
        gpio_set_level(IN2_PIN, 1);
    }

    uint32_t duty = (uint32_t)(fabs(speed));
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, CHANNEL_NUM, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set duty for LEDC channel 0");
        return ret;
    }
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, CHANNEL_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update duty for LEDC channel 0");
        return ret;
    }

    return ESP_OK;
}

esp_err_t L298N::onConfigUpdate(const IRuntimeConfig& config) {
    ESP_LOGI(TAG, "Updating L298N configuration");
    // Add any configuration update logic here
    return ESP_OK;
}

esp_err_t MX1616H::init(const IRuntimeConfig& config) {
    ESP_LOGI(TAG, "Initializing MX1616H motor driver");

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 25000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer");
        return ret;
    }

    ledc_channel_config_t ledc_channel[2] = {
        {
            .gpio_num       = IN1_PIN,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = CHANNEL_NUM_1,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0,
            .hpoint         = 0
        },
        {
            .gpio_num       = IN2_PIN,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = CHANNEL_NUM_2,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0,
            .hpoint         = 0
        }
    };

    for (int i = 0; i < 2; i++) {
        ret = ledc_channel_config(&ledc_channel[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC channel %d", i);
            return ret;
        }
    }

    ESP_LOGI(TAG, "MX1616H motor driver initialized successfully");
    return ESP_OK;
}

esp_err_t MX1616H::setSpeed(float speed) const {
    ESP_LOGD(TAG, "Setting MX1616H speed to %.2f", speed);

    uint32_t duty = (uint32_t)(fabs(speed * (1023-500))) + 500; //10 BIT PWM resolution minus - 500 TO ACCOUNT FOR THE DEADZONE FROM 0 - 500

    if (speed > 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CHANNEL_NUM_1, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CHANNEL_NUM_2, duty);
    } else if (speed < 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CHANNEL_NUM_1, duty);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CHANNEL_NUM_2, 0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CHANNEL_NUM_1, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CHANNEL_NUM_2, 0);
    }
        
    esp_err_t ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, CHANNEL_NUM_1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update duty for LEDC channel %d", CHANNEL_NUM_1);
        return ret;
    }

    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, CHANNEL_NUM_2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update duty for LEDC channel %d", CHANNEL_NUM_2);
        return ret;
    }   

    return ESP_OK;
}

esp_err_t MX1616H::onConfigUpdate(const IRuntimeConfig& config) {
    ESP_LOGI(TAG, "Updating MX1616H configuration");
    // Add any configuration update logic here
    return ESP_OK;
}
