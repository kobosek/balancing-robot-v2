#include "IMUHealthMonitor.hpp"

#include "IIMUFaultSink.hpp"
#include <algorithm>
#include "esp_log.h"

IMUHealthMonitor::IMUHealthMonitor(IIMUFaultSink& faultSink, const SystemBehaviorConfig& initialBehavior) :
    m_faultSink(faultSink),
    m_i2c_failure_threshold(5),
    m_no_data_failure_threshold(5),
    m_data_timeout_us(500000) {
    applyConfig(initialBehavior);
    const int64_t now = esp_timer_get_time();
    m_last_pet_time_us.store(now, std::memory_order_release);
}

void IMUHealthMonitor::applyConfig(const SystemBehaviorConfig& config) {
    m_i2c_failure_threshold = std::max<uint8_t>(1, static_cast<uint8_t>(config.imu_health_i2c_fail_threshold));
    m_no_data_failure_threshold = std::max<uint8_t>(1, static_cast<uint8_t>(config.imu_health_no_data_threshold));
    m_data_timeout_us = std::max<int64_t>(1000, static_cast<int64_t>(config.imu_health_data_timeout_ms) * 1000LL);
}

void IMUHealthMonitor::checkHealth() {
    if (!m_monitoring_enabled.load(std::memory_order_relaxed)) {
        return;
    }

    const int64_t currentTime = esp_timer_get_time();
    const int64_t lastPet = m_last_pet_time_us.load(std::memory_order_acquire);
    if ((currentTime - lastPet) <= m_data_timeout_us) {
        return;
    }

    const uint8_t noDataCount = m_no_data_counter.fetch_add(1, std::memory_order_relaxed) + 1;
    if (noDataCount >= m_no_data_failure_threshold) {
        m_no_data_counter.store(0, std::memory_order_relaxed);
        m_faultSink.onIMUHardFault(ESP_ERR_TIMEOUT);
    }
}

void IMUHealthMonitor::pet() {
    m_last_pet_time_us.store(esp_timer_get_time(), std::memory_order_release);
    m_no_data_counter.store(0, std::memory_order_relaxed);
    m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);
}

void IMUHealthMonitor::notifyIMUStateChange(IMUState newState) {
    const bool enableMonitoring = (newState == IMUState::OPERATIONAL);
    m_monitoring_enabled.store(enableMonitoring, std::memory_order_release);
    resetFaultCounters();
    m_last_pet_time_us.store(esp_timer_get_time(), std::memory_order_release);
}

bool IMUHealthMonitor::recordCommunicationFailure(esp_err_t errorCode) {
    (void)errorCode;
    const uint8_t failures = m_consecutive_i2c_failures.fetch_add(1, std::memory_order_relaxed) + 1;
    return failures >= m_i2c_failure_threshold;
}

void IMUHealthMonitor::resetFaultCounters() {
    m_no_data_counter.store(0, std::memory_order_relaxed);
    m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);
}
