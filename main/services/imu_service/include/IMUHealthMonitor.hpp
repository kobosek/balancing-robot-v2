#pragma once

#include "ConfigData.hpp"
#include "IMUState.hpp"
#include "esp_err.h"
#include "esp_timer.h"
#include <atomic>

class IIMUFaultSink;

class IMUHealthMonitor {
public:
    IMUHealthMonitor(IIMUFaultSink& faultSink, const SystemBehaviorConfig& initialBehavior);
    ~IMUHealthMonitor() = default;

    void checkHealth();
    void pet();
    void notifyIMUStateChange(IMUState newState);
    void applyConfig(const SystemBehaviorConfig& config);
    bool recordCommunicationFailure(esp_err_t errorCode);
    void resetFaultCounters();

private:
    static constexpr const char* TAG = "IMUHealthMonitor";

    IIMUFaultSink& m_faultSink;
    uint8_t m_i2c_failure_threshold;
    uint8_t m_no_data_failure_threshold;
    int64_t m_data_timeout_us;

    std::atomic<int64_t> m_last_pet_time_us{0};
    std::atomic<uint8_t> m_consecutive_i2c_failures{0};
    std::atomic<uint8_t> m_no_data_counter{0};
    std::atomic<bool> m_monitoring_enabled{false};
};
