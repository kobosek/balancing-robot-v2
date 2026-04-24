// main/EncoderService.cpp
#include "EncoderService.hpp"           // Relative path within module's include dir
#include "esp_check.h"
#include <cmath>
#include "esp_log.h"                    // Moved from header

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

EncoderService::EncoderService(const EncoderConfig& config) :
    m_config(config),
    m_unit_left(nullptr),
    m_unit_right(nullptr)
{
    // Pre-calculate conversion factor: (DEG/rev) / (pulses/rev_motor) / gear_ratio
    // DEG/rev = 360
    if (m_config.pulses_per_revolution_motor > 0 && m_config.gear_ratio > 0) {
        m_degs_per_pulse = 360.0f / (m_config.pulses_per_revolution_motor * m_config.gear_ratio); // <-- Changed calculation
        ESP_LOGI(TAG,"Encoder degrees per pulse calculated: %f", m_degs_per_pulse);
    } else {
        ESP_LOGE(TAG,"Invalid encoder config: pulses/rev or gear ratio is zero!");
        m_degs_per_pulse = 0.0f;
    }
    reset();
}

EncoderService::~EncoderService() {
    if (m_unit_left) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_stop(m_unit_left));
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_disable(m_unit_left));
    }
    if (m_unit_right) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_stop(m_unit_right));
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_disable(m_unit_right));
    }
    if (m_channel_left_a) { pcnt_del_channel(m_channel_left_a); }
    if (m_channel_left_b) { pcnt_del_channel(m_channel_left_b); }
    if (m_channel_right_a) { pcnt_del_channel(m_channel_right_a); }
    if (m_channel_right_b) { pcnt_del_channel(m_channel_right_b); }
    if (m_unit_left) { pcnt_del_unit(m_unit_left); }
    if (m_unit_right) { pcnt_del_unit(m_unit_right); }
}

void EncoderService::reset() {
    m_last_pulse_count_left = 0;
    m_last_pulse_count_right = 0;
    m_speed_dps_left = 0.0f; // Renamed
    m_speed_dps_right = 0.0f; // Renamed
     m_last_unfiltered_speed_left_dps = 0.0f; // Renamed
     m_last_unfiltered_speed_right_dps = 0.0f; // Renamed

    if (m_unit_left) pcnt_unit_clear_count(m_unit_left);
    if (m_unit_right) pcnt_unit_clear_count(m_unit_right);
    ESP_LOGI(TAG, "EncoderService state reset.");
}

esp_err_t EncoderService::init() {
    // ... (PCNT initialization remains the same) ...
    ESP_LOGI(TAG, "Initializing EncoderService...");
    esp_err_t ret;
    ret = initPCNTUnit(m_config.left_pin_a, m_config.left_pin_b, &m_unit_left, &m_channel_left_a, &m_channel_left_b);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Left Encoder PCNT");
    ESP_LOGI(TAG, "Left Encoder PCNT Initialized (Pins A:%d, B:%d)", m_config.left_pin_a, m_config.left_pin_b);
    ret = initPCNTUnit(m_config.right_pin_a, m_config.right_pin_b, &m_unit_right, &m_channel_right_a, &m_channel_right_b);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Right Encoder PCNT");
    ESP_LOGI(TAG, "Right Encoder PCNT Initialized (Pins A:%d, B:%d)", m_config.right_pin_a, m_config.right_pin_b);
    ESP_LOGI(TAG, "EncoderService Initialized Successfully.");
    return ESP_OK;
}

esp_err_t EncoderService::initPCNTUnit(int pinA, int pinB, pcnt_unit_handle_t* unit_handle,
                                       pcnt_channel_handle_t* channel_a_handle,
                                       pcnt_channel_handle_t* channel_b_handle) {
    // ... (PCNT initialization remains the same) ...
     ESP_LOGD(TAG, "Init PCNT Unit for pins A:%d, B:%d", pinA, pinB);
     *unit_handle = nullptr;
     *channel_a_handle = nullptr;
     *channel_b_handle = nullptr;
     pcnt_unit_handle_t unit = nullptr;
     pcnt_channel_handle_t pcnt_chan_a = nullptr;
     pcnt_channel_handle_t pcnt_chan_b = nullptr;
     bool unit_enabled = false;
     bool unit_started = false;
     auto cleanup = [&]() {
         if (unit_started) {
             ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_stop(unit));
         }
         if (unit_enabled) {
             ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_disable(unit));
         }
         if (pcnt_chan_a) {
             ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_del_channel(pcnt_chan_a));
         }
         if (pcnt_chan_b) {
             ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_del_channel(pcnt_chan_b));
         }
         if (unit) {
             ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_del_unit(unit));
         }
     };
     pcnt_unit_config_t unit_config = { .low_limit = m_config.pcnt_low_limit, .high_limit = m_config.pcnt_high_limit, .flags = { .accum_count = 1 } };
     esp_err_t ret = pcnt_new_unit(&unit_config, &unit);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed create PCNT unit: %s", esp_err_to_name(ret));
         return ret;
     }
     pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = (uint32_t)m_config.pcnt_filter_ns };
     ret = pcnt_unit_set_glitch_filter(unit, &filter_config);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed set PCNT glitch filter: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     pcnt_chan_config_t chan_a_config = { .edge_gpio_num = pinA, .level_gpio_num = pinB };
     ret = pcnt_new_channel(unit, &chan_a_config, &pcnt_chan_a);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed create PCNT channel A: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     pcnt_chan_config_t chan_b_config = { .edge_gpio_num = pinB, .level_gpio_num = pinA };
     ret = pcnt_new_channel(unit, &chan_b_config, &pcnt_chan_b);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed create PCNT channel B: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Chan A edge fail: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Chan A level fail: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Chan B edge fail: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Chan B level fail: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_unit_enable(unit);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed enable PCNT unit: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     unit_enabled = true;
     ret = pcnt_unit_clear_count(unit);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed clear PCNT count: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_unit_start(unit);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed start PCNT unit: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     unit_started = true;
     *unit_handle = unit;
     *channel_a_handle = pcnt_chan_a;
     *channel_b_handle = pcnt_chan_b;
     return ESP_OK;
}


void EncoderService::update(float dt) {
    if (dt <= 0 || m_degs_per_pulse == 0.0f) { return; } // Check new constant name

    // Left Encoder
    if (m_unit_left) {
        int count_left = 0;
        if (pcnt_unit_get_count(m_unit_left, &count_left) == ESP_OK) {
            int32_t delta_pulses = calculateDeltaPulses(count_left, m_last_pulse_count_left);
            float instant_speed_dps = static_cast<float>(delta_pulses) * m_degs_per_pulse / dt; // Calculate DPS
             m_speed_dps_left = m_config.speed_filter_alpha * instant_speed_dps + (1.0f - m_config.speed_filter_alpha) * m_speed_dps_left; // Rename state var
             m_last_pulse_count_left = count_left;
             m_last_unfiltered_speed_left_dps = instant_speed_dps; // Rename state var
        } else { ESP_LOGE(TAG, "Failed read left encoder"); }
    } else { m_speed_dps_left = 0.0f; } // Rename state var

    // Right Encoder
     if (m_unit_right) {
        int count_right = 0;
        if (pcnt_unit_get_count(m_unit_right, &count_right) == ESP_OK) {
            int32_t delta_pulses = calculateDeltaPulses(count_right, m_last_pulse_count_right);
            float instant_speed_dps = static_cast<float>(delta_pulses) * m_degs_per_pulse / dt; // Calculate DPS
            m_speed_dps_right = m_config.speed_filter_alpha * instant_speed_dps + (1.0f - m_config.speed_filter_alpha) * m_speed_dps_right; // Rename state var
            m_last_pulse_count_right = count_right;
            m_last_unfiltered_speed_right_dps = instant_speed_dps; // Rename state var
        } else { ESP_LOGE(TAG, "Failed read right encoder"); }
    } else { m_speed_dps_right = 0.0f; } // Rename state var

     ESP_LOGV(TAG, "Update: dt=%.4f | LSpd: %.1f (%.1f) RSpd: %.1f (%.1f) dps", // Update log unit
              dt, m_speed_dps_left, m_last_unfiltered_speed_left_dps,
              m_speed_dps_right, m_last_unfiltered_speed_right_dps);
}

int32_t EncoderService::calculateDeltaPulses(int currentCount, int previousCount) const {
    int32_t delta = static_cast<int32_t>(currentCount) - static_cast<int32_t>(previousCount);
    const int32_t range =
        static_cast<int32_t>(m_config.pcnt_high_limit) - static_cast<int32_t>(m_config.pcnt_low_limit) + 1;
    if (range <= 0) {
        return delta;
    }

    const int32_t halfRange = range / 2;
    if (delta > halfRange) {
        delta -= range;
    } else if (delta < -halfRange) {
        delta += range;
    }

    return delta;
}
