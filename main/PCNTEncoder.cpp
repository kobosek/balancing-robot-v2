#include "include/Encoder.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#define BDC_ENCODER_PCNT_HIGH_LIMIT   30000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -30000

esp_err_t PCNTEncoder::init(const IRuntimeConfig& config) {

    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");
    pcnt_unit_config_t unit_config = {
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .intr_priority = 3,
        .flags { .accum_count = 1 }
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &m_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(m_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = m_pinA,
        .level_gpio_num = m_pinB,
    };
    
    pcnt_channel_handle_t pcnt_chan_a = NULL;

    ESP_ERROR_CHECK(pcnt_new_channel(m_unit, &chan_a_config, &pcnt_chan_a));
    
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = m_pinB,
        .level_gpio_num = m_pinA,
    };

    pcnt_channel_handle_t pcnt_chan_b = NULL;

    ESP_ERROR_CHECK(pcnt_new_channel(m_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(m_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(m_unit, BDC_ENCODER_PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(m_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(m_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(m_unit));

    return ESP_OK;
}

const float PULSES_PER_REVOLUTION = 28.0f;
const float DEGREES_PER_REVOLUTION = 360.0f;
const float GEAR_RATIO = 100.0f;

float PCNTEncoder::getSpeed(float dt) {
    int pulseCount = 0;
    pcnt_unit_get_count(m_unit, &pulseCount);

    float deltaPos = float (pulseCount - lastPulseCount);
    float instantSpeed = deltaPos / PULSES_PER_REVOLUTION * DEGREES_PER_REVOLUTION / GEAR_RATIO / dt;
    lastPulseCount = pulseCount;     

    float instantAccel = abs(instantSpeed - last_speed) / dt;
    float baseAlpha = 0.05f;
    float accelScale = std::min(1.0f, instantAccel / 10000.0f);  // Adjust 1000 based on your max accel
    float alpha = baseAlpha + (0.5f - baseAlpha) * accelScale;

    float filteredSpeed = alpha * instantSpeed + (1.0f - alpha) * last_filtered_speed;
    
    last_speed = instantSpeed;
    last_filtered_speed = filteredSpeed;
    return last_filtered_speed;

};

esp_err_t PCNTEncoder::onConfigUpdate(const IRuntimeConfig& config) {
    ESP_LOGI(TAG, "Updating Encoder configuration");
    // Add any configuration update logic here
    return ESP_OK;
}