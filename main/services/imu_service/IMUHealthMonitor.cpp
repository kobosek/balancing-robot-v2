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
    m_i2c_failure_threshold.store(
        std::max<uint8_t>(1, static_cast<uint8_t>(config.imu_health_i2c_fail_threshold)),
        std::memory_order_release);
    m_no_data_failure_threshold.store(
        std::max<uint8_t>(1, static_cast<uint8_t>(config.imu_health_no_data_threshold)),
        std::memory_order_release);
    m_data_timeout_us.store(
        std::max<int64_t>(1000, static_cast<int64_t>(config.imu_health_data_timeout_ms) * 1000LL),
        std::memory_order_release);
}

void IMUHealthMonitor::checkHealth() {
    if (!m_monitoring_enabled.load(std::memory_order_relaxed)) {
        return;
    }

    const int64_t currentTime = esp_timer_get_time();
    const int64_t lastPet = m_last_pet_time_us.load(std::memory_order_acquire);
    const int64_t dataTimeoutUs = m_data_timeout_us.load(std::memory_order_acquire);
    if ((currentTime - lastPet) <= dataTimeoutUs) {
        return;
    }

    const uint8_t noDataCount = m_no_data_counter.fetch_add(1, std::memory_order_relaxed) + 1;
    const uint8_t noDataThreshold = m_no_data_failure_threshold.load(std::memory_order_acquire);
    if (noDataCount >= noDataThreshold) {
        ESP_LOGE(TAG,
                 "IMU data timeout threshold reached (misses=%u threshold=%u overdue_us=%lld timeout_us=%lld)",
                 static_cast<unsigned>(noDataCount),
                 static_cast<unsigned>(noDataThreshold),
                 static_cast<long long>(currentTime - lastPet),
                 static_cast<long long>(dataTimeoutUs));
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
    const uint8_t failures = m_consecutive_i2c_failures.fetch_add(1, std::memory_order_relaxed) + 1;
    const uint8_t failureThreshold = m_i2c_failure_threshold.load(std::memory_order_acquire);
    if (failures >= failureThreshold) {
        ESP_LOGE(TAG,
                 "IMU communication failure threshold reached (code=%s count=%u threshold=%u)",
                 esp_err_to_name(errorCode),
                 static_cast<unsigned>(failures),
                 static_cast<unsigned>(failureThreshold));
        return true;
    }

    ESP_LOGW(TAG,
             "IMU communication failure recorded (code=%s count=%u threshold=%u)",
             esp_err_to_name(errorCode),
             static_cast<unsigned>(failures),
             static_cast<unsigned>(failureThreshold));
    return false;
}

void IMUHealthMonitor::resetFaultCounters() {
    m_no_data_counter.store(0, std::memory_order_relaxed);
    m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);
}
