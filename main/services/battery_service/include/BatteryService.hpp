#pragma once

#include "ConfigData.hpp"
#include "EventBus.hpp"
#include "BATTERY_StatusUpdate.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "EventHandler.hpp"             // For EventHandler base class
#include "esp_log.h"
#include <mutex>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Forward declarations
class BaseEvent;
class CONFIG_FullConfigUpdate;

class BatteryService : public EventHandler {
public:
    // Constructor now takes initial config structs
    BatteryService(const BatteryConfig& initialConfig, const SystemBehaviorConfig& initialBehavior, EventBus& bus);
    ~BatteryService();

    esp_err_t init();
    void updateBatteryStatus();
    BatteryStatus getLatestStatus() const;
    // TickType_t getMonitoringInterval() const; // No longer needed, Task takes interval

    // Event handling via EventHandler interface
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }
    
    // Keep for backwards compatibility during transition
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "BatteryService";

    // Store local copies of config structs
    BatteryConfig m_config;
    SystemBehaviorConfig m_behaviorConfig;

    EventBus& m_eventBus;
    // ConfigurationService& m_configService; // REMOVE

    adc_channel_t m_adc_channel;
    adc_cali_handle_t m_adc_cali_handle;
    bool m_adc_cali_enable;
    adc_oneshot_unit_handle_t m_oneshot_adc_handle;

    // Local copies of relevant config values (updated by handler)
    int m_oversampling_count;
    // TickType_t m_read_interval_ticks; // Task gets interval directly

    mutable std::mutex m_status_mutex;
    BatteryStatus m_latest_status;

    bool adc_calibration_init(adc_channel_t channel);
    esp_err_t map_gpio_to_channel();
    // void loadConfigParameters(); // REMOVE or make private apply
    void applyConfig(const BatteryConfig& batConf, const SystemBehaviorConfig& behaviorConf); // Add apply helper
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event); // Specific event handler
};