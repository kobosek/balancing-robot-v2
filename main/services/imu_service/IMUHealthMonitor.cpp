#include "IMUHealthMonitor.hpp"
#include "mpu6050.hpp"
#include "EventBus.hpp"
#include "CONFIG_FullConfigUpdate.hpp" // Include event with payload
#include "IMU_CommunicationError.hpp"
#include "EventTypes.hpp"
#include "IMU_CalibrationStarted.hpp" // For event type check, could use BaseEvent with type check
#include "IMU_CalibrationCompleted.hpp" // For event type check
#include "IMU_StateChanged.hpp" // To react to IMU state changes
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Needed for SystemBehaviorConfig
#include "IMUService.hpp" // Include for IMUState enum and stateToString
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm> // For std::max

// Constructor takes initial SystemBehaviorConfig
IMUHealthMonitor::IMUHealthMonitor(MPU6050Driver& driver, EventBus& bus, const SystemBehaviorConfig& initialBehavior) :
    m_driver(driver),
    m_eventBus(bus),
    // Initialize thresholds with temporary values before applyConfig
    m_i2c_failure_threshold(5),
    m_no_data_failure_threshold(5),
    m_data_timeout_us(500000),
    m_proactive_check_interval_us(10000000),
    m_limited_checks(false)
{
    applyConfig(initialBehavior); // Apply initial config
    int64_t now = esp_timer_get_time();
    m_last_pet_time_us.store(now, std::memory_order_release);
    m_last_proactive_check_time_us.store(now, std::memory_order_release);
    ESP_LOGI(TAG, "IMUHealthMonitor Initialized.");
}

// Apply config values from struct
void IMUHealthMonitor::applyConfig(const SystemBehaviorConfig& config) {
    m_i2c_failure_threshold = std::max((uint8_t)1, (uint8_t)config.imu_health_i2c_fail_threshold);
    m_no_data_failure_threshold = std::max((uint8_t)1, (uint8_t)config.imu_health_no_data_threshold);
    m_data_timeout_us = std::max((int64_t)1000, (int64_t)config.imu_health_data_timeout_ms * 1000LL);
    m_proactive_check_interval_us = std::max((int64_t)1000, (int64_t)config.imu_health_proactive_check_ms * 1000LL);

    ESP_LOGI(TAG, "Applied IMUHealthMonitor params: I2CFailThresh=%d, NoDataThresh=%d, Timeout=%lldus, ProactiveCheck=%lldus",
             m_i2c_failure_threshold, m_no_data_failure_threshold, m_data_timeout_us, m_proactive_check_interval_us);
}

void IMUHealthMonitor::pet() {
    // Called by IMUService after successfully processing a data batch
    m_last_pet_time_us.store(esp_timer_get_time(), std::memory_order_release);
    // Reset counters on successful pet, indicating data flow is okay
    m_no_data_counter.store(0, std::memory_order_relaxed);
}

void IMUHealthMonitor::subscribeToEvents(EventBus& bus) {
    // Subscribe to IMU_StateChanged events to adapt behavior
    bus.subscribe(EventType::IMU_STATE_CHANGED, [this](const BaseEvent& ev){
        const auto& stateEvent = static_cast<const IMU_StateChanged&>(ev);
        this->notifyIMUStateChange(stateEvent.newState);
    });
    // Subscribe to CONFIG_UPDATED to reload thresholds
    bus.subscribe(EventType::CONFIG_FULL_UPDATE, [this](const BaseEvent& ev){
        this->handleConfigUpdate(ev);
    });
    ESP_LOGI(TAG, "Subscribed to IMU_STATE_CHANGED and CONFIG_FULL_UPDATE events.");
}

// Handle config update event
void IMUHealthMonitor::handleConfigUpdate(const BaseEvent& event) {
    // Basic type check, could be more robust if BaseEvent had a dynamic_cast safe way
    if (event.type != EventType::CONFIG_FULL_UPDATE) return;
    ESP_LOGD(TAG, "Handling config update event.");
    const auto& configEvent = static_cast<const CONFIG_FullConfigUpdate&>(event);
    applyConfig(configEvent.configData.behavior);
}


// Methods for state machine coordination
void IMUHealthMonitor::notifyIMUStateChange(IMUState newState) {
    ESP_LOGI(TAG, "IMU state change notification received: %s", IMUService::stateToString(newState));
        
    // Set limited checks based on the IMU state
    // INITIALIZED, OPERATIONAL are full monitoring
    // CALIBRATION, RECOVERY are limited checks
    if (newState == IMUState::INITIALIZED || newState == IMUState::OPERATIONAL) {
        setLimitedHealthChecks(false);
        ESP_LOGI(TAG, "Full health monitoring enabled for state %s", IMUService::stateToString(newState));
    } 
    else if (newState == IMUState::CALIBRATION || newState == IMUState::RECOVERY) {
        setLimitedHealthChecks(true);
        ESP_LOGI(TAG, "Limited health checks enabled for state %s", IMUService::stateToString(newState));
    }
    else {
        ESP_LOGW(TAG, "Unknown IMU state in notification: %d", static_cast<int>(newState));
    }

    // Reset counters to prevent false triggers immediately after state change (especially exiting limited checks)
    m_no_data_counter.store(0, std::memory_order_relaxed);
    m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);
    // Reset the last_pet_time to avoid potential false positive timeout when full monitoring resumes
    m_last_pet_time_us.store(esp_timer_get_time(), std::memory_order_release);
}

void IMUHealthMonitor::setLimitedHealthChecks(bool limitChecks) {
    bool oldValue = m_limited_checks.exchange(limitChecks, std::memory_order_acq_rel);
    if (oldValue != limitChecks) {
        ESP_LOGI(TAG, "Health monitor mode changed: %s", 
                limitChecks ? "LIMITED CHECKS (proactive I2C only)" : "FULL MONITORING (I2C + data timeout)");
        
        // Reset counters when changing modes
        m_no_data_counter.store(0, std::memory_order_relaxed);
        m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);
        
        // Reset the last_pet_time when enabling full monitoring
        if (!limitChecks) {
            m_last_pet_time_us.store(esp_timer_get_time(), std::memory_order_release);
        }
    }
}

void IMUHealthMonitor::checkHealth() {
    int64_t current_time = esp_timer_get_time();
    int64_t last_proactive_check = m_last_proactive_check_time_us.load(std::memory_order_acquire);
    bool is_proactive_check_time = (current_time - last_proactive_check) > m_proactive_check_interval_us;

    // --- Check if in limited checks mode (calibration or recovery) ---
    if (m_limited_checks.load(std::memory_order_relaxed)) {
        // Only perform proactive I2C check if it's time
        if (is_proactive_check_time) {
            ESP_LOGD(TAG, "Performing proactive IMU health check (LIMITED MODE)...");
            m_last_proactive_check_time_us.store(current_time, std::memory_order_release);

            uint8_t who_am_i = 0;
            esp_err_t ret = m_driver.readRegisters(MPU6050Register::WHO_AM_I, &who_am_i, 1);

            if (ret != ESP_OK) {
                uint8_t failures = m_consecutive_i2c_failures.fetch_add(1, std::memory_order_relaxed) + 1;
                ESP_LOGE(TAG, "IMU comm error during limited health check: %s (%d failures)", esp_err_to_name(ret), failures);
                if (failures >= m_i2c_failure_threshold) {
                    ESP_LOGE(TAG, "IMU I2C failure threshold (%d) reached during limited check. Publishing event.", failures);
                    m_eventBus.publish(IMU_CommunicationError(ret));
                    m_consecutive_i2c_failures.store(0, std::memory_order_relaxed); 
                    m_sensor_disconnected.store(true, std::memory_order_relaxed);
                }
            } else if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
                ESP_LOGE(TAG, "IMU WHO_AM_I mismatch during limited health check! Expected 0x%02X, Got 0x%02X. Publishing event.",
                         MPU6050_WHO_AM_I_VALUE, who_am_i);
                m_eventBus.publish(IMU_CommunicationError(ESP_ERR_INVALID_RESPONSE));
                m_sensor_disconnected.store(true, std::memory_order_relaxed);
                m_consecutive_i2c_failures.store(0, std::memory_order_relaxed); 
            } else {
                // Communication OK during limited proactive check
                m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);
                if (m_sensor_disconnected.load(std::memory_order_relaxed)) {
                    ESP_LOGI(TAG, "IMU reconnected successfully (during limited check).");
                    m_sensor_disconnected.store(false, std::memory_order_relaxed);
                }
            }
        } else {
             ESP_LOGV(TAG, "Skipping health check during limited mode (not proactive check time).");
        }
        return; 
    }

    // --- Normal Health Check Logic (Not in limited mode) ---
    int64_t last_pet = m_last_pet_time_us.load(std::memory_order_acquire);
    bool is_timeout = (current_time - last_pet) > m_data_timeout_us;

    if (!is_timeout && !is_proactive_check_time) {
        return;
    }

    if (is_proactive_check_time) {
        ESP_LOGD(TAG, "Performing proactive IMU health check (FULL MODE)...");
        m_last_proactive_check_time_us.store(current_time, std::memory_order_release);
    }
    if (is_timeout) {
        ESP_LOGW(TAG, "IMU data timeout detected! Last pet: %.2f seconds ago.", (current_time - last_pet) / 1000000.0f);
    }

    uint8_t who_am_i = 0;
    esp_err_t ret = m_driver.readRegisters(MPU6050Register::WHO_AM_I, &who_am_i, 1);

    if (ret != ESP_OK) {
        uint8_t failures = m_consecutive_i2c_failures.fetch_add(1, std::memory_order_relaxed) + 1;
        ESP_LOGE(TAG, "IMU comm error during full health check: %s (%d failures)", esp_err_to_name(ret), failures);
        if (failures >= m_i2c_failure_threshold) {
            ESP_LOGE(TAG, "IMU I2C failure threshold (%d) reached. Publishing event.", failures);
            m_eventBus.publish(IMU_CommunicationError(ret));
            m_consecutive_i2c_failures.store(0, std::memory_order_relaxed); 
            m_sensor_disconnected.store(true, std::memory_order_relaxed);
        }
        m_no_data_counter.store(0, std::memory_order_relaxed); 
        return; 
    }

    m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);

    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "IMU WHO_AM_I mismatch! Expected 0x%02X, Got 0x%02X. Publishing event.",
                 MPU6050_WHO_AM_I_VALUE, who_am_i);
        m_eventBus.publish(IMU_CommunicationError(ESP_ERR_INVALID_RESPONSE));
        m_sensor_disconnected.store(true, std::memory_order_relaxed);
        return; 
    }

    if (m_sensor_disconnected.load(std::memory_order_relaxed)) {
         ESP_LOGI(TAG, "IMU reconnected successfully.");
         m_sensor_disconnected.store(false, std::memory_order_relaxed);
    }

    if (is_timeout) {
        uint8_t no_data_count = m_no_data_counter.fetch_add(1, std::memory_order_relaxed) + 1;
        ESP_LOGW(TAG, "IMU responsive but no data received (Timeout: Yes, Comm: OK). No-data count: %d", no_data_count);
        if (no_data_count >= m_no_data_failure_threshold) {
            ESP_LOGE(TAG, "IMU no-data threshold (%d) reached. Publishing error event.", no_data_count);
            m_eventBus.publish(IMU_CommunicationError(ESP_ERR_TIMEOUT)); // ESP_ERR_TIMEOUT indicates data flow issue
            m_no_data_counter.store(0, std::memory_order_relaxed); 
        }
    } else {
        m_no_data_counter.store(0, std::memory_order_relaxed);
    }
}