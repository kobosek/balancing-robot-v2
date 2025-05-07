#pragma once

#include "esp_err.h"
#include "esp_timer.h"
#include <atomic>
#include "ConfigData.hpp" // Include full definition

// Forward Declarations
class MPU6050Driver;
class EventBus;
class BaseEvent;
class CONFIG_FullConfigUpdate;
class IMU_CommunicationError; // Forward declaration for communication error event
enum class IMUState; // Add forward declaration for IMU state

class IMUHealthMonitor {
public:
    // Constructor now takes initial SystemBehaviorConfig
    IMUHealthMonitor(MPU6050Driver& driver, EventBus& bus, const SystemBehaviorConfig& initialBehavior);
    ~IMUHealthMonitor() = default;

    void checkHealth();
    void pet();
    
    // Method to inform the health monitor about IMU state changes
    void notifyIMUStateChange(IMUState newState);
    
    // Set operational mode based on IMU state
    void setLimitedHealthChecks(bool limitChecks);

    // Public constant for WHO_AM_I check (used by IMUService recovery)
    static constexpr uint8_t MPU6050_WHO_AM_I_VALUE = 0x68;

    // Method to subscribe to necessary events
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "IMUHealthMonitor";

    MPU6050Driver& m_driver;
    EventBus& m_eventBus;

    // Internal state loaded from config
    uint8_t m_i2c_failure_threshold;
    uint8_t m_no_data_failure_threshold;
    int64_t m_data_timeout_us;
    int64_t m_proactive_check_interval_us;

    std::atomic<int64_t> m_last_pet_time_us{0};
    std::atomic<int64_t> m_last_proactive_check_time_us{0};
    std::atomic<uint8_t> m_consecutive_i2c_failures{0};
    std::atomic<uint8_t> m_no_data_counter{0};
    std::atomic<bool> m_sensor_disconnected{false};
    std::atomic<bool> m_limited_checks{false}; // Renamed from m_calibration_active for clarity

    // Event Handlers
    void handleCalibrationStarted(const BaseEvent& ev);
    void handleCalibrationComplete(const BaseEvent& ev);
    void handleConfigUpdate(const BaseEvent& event);

    // Helper to apply config values
    void applyConfig(const SystemBehaviorConfig& config);
};