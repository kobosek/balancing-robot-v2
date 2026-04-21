// main/include/EncoderService.hpp
#pragma once

#include "ConfigData.hpp"               // Found via INCLUDE_DIRS
#include "driver/pulse_cnt.h"
// #include "esp_log.h" // Moved to .cpp
#include <cmath>                        // For M_PI if needed

class EncoderService {
public:
    EncoderService(const EncoderConfig& config);
    ~EncoderService();

    esp_err_t init();
    void update(float dt);
    void reset();

    // --- Updated Getters to return DPS ---
    float getLeftSpeedDegPerSec() const { return m_speed_dps_left; } // Renamed getter
    float getRightSpeedDegPerSec() const { return m_speed_dps_right; } // Renamed getter
    // --- End Update ---

private:
    static constexpr const char* TAG = "EncoderService";
    const EncoderConfig m_config;

    pcnt_unit_handle_t m_unit_left = nullptr;
    pcnt_unit_handle_t m_unit_right = nullptr;

    volatile int m_last_pulse_count_left = 0;
    volatile int m_last_pulse_count_right = 0;
    // --- Updated state variables ---
    volatile float m_speed_dps_left = 0.0f; // Renamed
    volatile float m_speed_dps_right = 0.0f; // Renamed
    volatile float m_last_unfiltered_speed_left_dps = 0.0f; // Renamed
    volatile float m_last_unfiltered_speed_right_dps = 0.0f; // Renamed
    float m_degs_per_pulse = 0.0f; // Renamed constant
    // --- End Update ---

    esp_err_t initPCNTUnit(int pinA, int pinB, pcnt_unit_handle_t* unit_handle);
};