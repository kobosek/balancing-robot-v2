#include "IMUHealthMonitor.hpp"
#include "mpu6050.hpp" // Need MPU6050Driver definition
#include "EventBus.hpp"
#include "IMU_CommunicationErrorEvent.hpp" // Event to publish
#include "esp_log.h"
#include "esp_timer.h"

IMUHealthMonitor::IMUHealthMonitor(MPU6050Driver& driver, EventBus& bus) :
    m_driver(driver),
    m_eventBus(bus)
{
    // Initialize timestamps
    int64_t now = esp_timer_get_time();
    m_last_pet_time_us.store(now, std::memory_order_release);
    m_last_proactive_check_time_us.store(now, std::memory_order_release);
    ESP_LOGI(TAG, "IMUHealthMonitor Initialized.");
}

void IMUHealthMonitor::pet() {
    // Called by IMUService after successfully processing a data batch
    m_last_pet_time_us.store(esp_timer_get_time(), std::memory_order_release);
    // Reset counters on successful pet, indicating data flow is okay
    m_no_data_counter.store(0, std::memory_order_relaxed);
}

void IMUHealthMonitor::checkHealth() {
    int64_t current_time = esp_timer_get_time();
    int64_t last_pet = m_last_pet_time_us.load(std::memory_order_acquire);
    int64_t last_proactive_check = m_last_proactive_check_time_us.load(std::memory_order_acquire);

    bool is_timeout = (current_time - last_pet) > DATA_TIMEOUT_US;
    bool is_proactive_check_time = (current_time - last_proactive_check) > PROACTIVE_CHECK_INTERVAL_US;

    // Only proceed if timed out or it's time for a proactive check
    if (!is_timeout && !is_proactive_check_time) {
        return;
    }

    if (is_proactive_check_time) {
        ESP_LOGD(TAG, "Performing proactive IMU health check...");
        m_last_proactive_check_time_us.store(current_time, std::memory_order_release);
    }
    if (is_timeout) {
        ESP_LOGW(TAG, "IMU data timeout detected! Last pet: %.2f seconds ago.", (current_time - last_pet) / 1000000.0f);
    }

    // --- Perform I2C Communication Check ---
    uint8_t who_am_i = 0;
    esp_err_t ret = m_driver.readRegisters(MPU6050Register::WHO_AM_I, &who_am_i, 1);

    if (ret != ESP_OK) {
        // Communication Error
        uint8_t failures = m_consecutive_i2c_failures.fetch_add(1, std::memory_order_relaxed) + 1;
        ESP_LOGE(TAG, "IMU comm error during health check: %s (%d failures)", esp_err_to_name(ret), failures);

        if (failures >= I2C_FAILURE_THRESHOLD) {
            ESP_LOGE(TAG, "IMU I2C failure threshold (%d) reached. Publishing event.", failures);
            m_eventBus.publish(IMU_CommunicationErrorEvent(ret));
            m_consecutive_i2c_failures.store(0, std::memory_order_relaxed); // Reset after publish
            m_sensor_disconnected.store(true, std::memory_order_relaxed); // Mark as disconnected
        }
        m_no_data_counter.store(0, std::memory_order_relaxed); // Not a 'no data' issue
        return; // Exit check early on comm failure
    }

    // --- Communication Successful ---
    m_consecutive_i2c_failures.store(0, std::memory_order_relaxed); // Reset counter

    // Check WHO_AM_I value
    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "IMU WHO_AM_I mismatch! Expected 0x%02X, Got 0x%02X. Publishing event.",
                 MPU6050_WHO_AM_I_VALUE, who_am_i);
        m_eventBus.publish(IMU_CommunicationErrorEvent(ESP_ERR_INVALID_RESPONSE));
        m_sensor_disconnected.store(true, std::memory_order_relaxed); // Mark as disconnected
        return; // Exit check early
    }

    // If we reach here, communication is OK and WHO_AM_I is correct
    if (m_sensor_disconnected.load(std::memory_order_relaxed)) {
         ESP_LOGI(TAG, "IMU reconnected successfully.");
         m_sensor_disconnected.store(false, std::memory_order_relaxed); // Mark as connected
    }

    // --- Check for 'Responsive but No Data' Scenario ---
    if (is_timeout) {
        // Communication is OK, but we haven't received data recently
        uint8_t no_data_count = m_no_data_counter.fetch_add(1, std::memory_order_relaxed) + 1;
        ESP_LOGW(TAG, "IMU responsive but no data received (Timeout: Yes, Comm: OK). No-data count: %d", no_data_count);

        if (no_data_count >= NO_DATA_FAILURE_THRESHOLD) {
            ESP_LOGE(TAG, "IMU no-data threshold (%d) reached. Publishing error event.", no_data_count);
            // Publish a generic communication error, IMUService will handle recovery
            m_eventBus.publish(IMU_CommunicationErrorEvent(ESP_ERR_TIMEOUT)); // Use Timeout error code
            m_no_data_counter.store(0, std::memory_order_relaxed); // Reset after publish
        }
    } else {
        // Not timed out, reset the no_data counter
        m_no_data_counter.store(0, std::memory_order_relaxed);
    }
}