#pragma once

#include "esp_err.h"
#include "esp_timer.h"
#include <atomic>

// Forward Declarations
class MPU6050Driver;
class EventBus;

class IMUHealthMonitor {
public:
    IMUHealthMonitor(MPU6050Driver& driver, EventBus& bus);
    ~IMUHealthMonitor() = default;

    void checkHealth();
    void pet();

    // Configuration constants (could be moved to ConfigData if needed)
    static constexpr uint8_t I2C_FAILURE_THRESHOLD = 5;
    static constexpr uint8_t NO_DATA_FAILURE_THRESHOLD = 5;
    static constexpr int64_t DATA_TIMEOUT_US = 500 * 1000; // 500ms
    static constexpr int64_t PROACTIVE_CHECK_INTERVAL_US = 10 * 1000 * 1000; // 10 seconds
    static constexpr uint8_t MPU6050_WHO_AM_I_VALUE = 0x68; // Expected WHO_AM_I

private:
    static constexpr const char* TAG = "IMUHealthMonitor";

    MPU6050Driver& m_driver;
    EventBus& m_eventBus;

    std::atomic<int64_t> m_last_pet_time_us{0};
    std::atomic<int64_t> m_last_proactive_check_time_us{0};
    std::atomic<uint8_t> m_consecutive_i2c_failures{0};
    std::atomic<uint8_t> m_no_data_counter{0}; // Counts 'responsive but no data'
    std::atomic<bool> m_sensor_disconnected{false}; // Tracks if fully disconnected
};