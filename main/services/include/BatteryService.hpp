// main/services/include/BatteryService.hpp
#pragma once

#include "ConfigData.hpp"
#include "EventBus.hpp"
#include "BatteryStatusUpdatedEvent.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <mutex>

// --- Add back required FreeRTOS includes ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// --- End Add back ---

class BatteryService {
public:
    BatteryService(const BatteryConfig& config, EventBus& bus);
    ~BatteryService();

    esp_err_t init();
    
    // Methods for task management
    void updateBatteryStatus(); // Now public for BatteryMonitorTask

    BatteryStatus getLatestStatus() const;
    
    // Get the configured monitoring interval
    static constexpr TickType_t getMonitoringInterval() { return READ_INTERVAL_MS; }

private:
    static constexpr const char* TAG = "BatteryService";
    static constexpr adc_bitwidth_t ADC_WIDTH = ADC_BITWIDTH_12;
    static constexpr adc_atten_t ADC_ATTEN = ADC_ATTEN_DB_12;
    static constexpr int OVERSAMPLING = 64;
    static constexpr TickType_t READ_INTERVAL_MS = 5000;

    const BatteryConfig m_config;
    EventBus& m_eventBus;

    adc_channel_t m_adc_channel;
    adc_cali_handle_t m_adc_cali_handle;
    bool m_adc_cali_enable;
    adc_oneshot_unit_handle_t m_oneshot_adc_handle;

    mutable std::mutex m_status_mutex;
    BatteryStatus m_latest_status;

    bool adc_calibration_init(adc_channel_t channel);
    esp_err_t map_gpio_to_channel();
};