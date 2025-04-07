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
    void start();
    void stop();

    BatteryStatus getLatestStatus() const;

private:
    static constexpr const char* TAG = "BatteryService";
    static constexpr adc_bitwidth_t ADC_WIDTH = ADC_BITWIDTH_12;
    static constexpr adc_atten_t ADC_ATTEN = ADC_ATTEN_DB_12;
    static constexpr int OVERSAMPLING = 64;
    static constexpr TickType_t READ_INTERVAL_MS = 5000; // TickType_t is now defined

    const BatteryConfig m_config;
    EventBus& m_eventBus;

    adc_channel_t m_adc_channel;
    adc_cali_handle_t m_adc_cali_handle;
    bool m_adc_cali_enable;
    TaskHandle_t m_monitor_task_handle; // TaskHandle_t is now defined
    adc_oneshot_unit_handle_t m_oneshot_adc_handle;

    mutable std::mutex m_status_mutex;
    BatteryStatus m_latest_status;

    static void monitorTaskWrapper(void* arg);
    void monitorTask();
    bool adc_calibration_init(adc_channel_t channel);
    void updateBatteryStatus();
    esp_err_t map_gpio_to_channel();
};