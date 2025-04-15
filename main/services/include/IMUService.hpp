#pragma once

#include "mpu6050.hpp"
#include "ConfigData.hpp"
#include "EventBus.hpp"
#include "OrientationEstimator.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <vector> 

// Forward declarations
class BaseEvent;
class StartCalibrationRequestEvent;
class CalibrationCompleteEvent;
class AttemptImuRecoveryCommand;

class IMUService {
public:
    IMUService(const MPU6050Config& config, EventBus& bus, OrientationEstimator& estimator);
    ~IMUService();

    esp_err_t init();
    esp_err_t triggerCalibration();

    // Recovery Methods
    esp_err_t resetSensor();
    esp_err_t reinitializeSensor();
    esp_err_t verifyCommunication();
    
    // Attempt recovery process (called via event)
    void attemptRecovery();
    
    // Methods exposed for task classes
    void processFifoData();
    void checkImuHealth();
    void petWatchdog();

private:
    static constexpr const char* TAG = "IMUService";

    // Data validation
    struct IMUData {
        struct {
            float x, y, z;
        } gyro, accel;
    };
    bool isDataConsistent(const IMUData& data);
    void handleStartCalibrationRequest(const BaseEvent& event);
    void handleAttemptRecoveryCommand(const AttemptImuRecoveryCommand& event);

    static void IRAM_ATTR isrHandler(void* arg);
    void handleInterrupt();

    esp_err_t performCalibration();

    const MPU6050Config m_config;
    EventBus& m_eventBus;
    OrientationEstimator& m_estimator;
    MPU6050 m_sensor;

    SemaphoreHandle_t m_calibration_mutex;
    volatile bool m_is_calibrating; // Should be accessed carefully across tasks/ISRs

    // Store offsets directly in DPS (Degrees Per Second)
    float m_gyro_offset_dps[3]; // Index 0:X, 1:Y, 2:Z

    std::atomic<uint8_t> m_isr_data_counter; // Counter for FIFO samples read by ISR

    std::atomic<int64_t> m_last_successful_read_timestamp_us{0};
    std::atomic<int64_t> m_last_proactive_check_time_us{0};
    bool m_watchdog_enabled;
    std::atomic<bool> m_watchdog_reset_flag{false}; // Flag to indicate watchdog triggered a reset
    std::atomic<uint8_t> m_consecutive_i2c_failures{0};
    std::atomic<uint8_t> m_no_data_counter{0}; // Counter for 'responsive but no data' occurrences
    std::atomic<int64_t> m_last_disconnect_time_us{0}; // Timestamp of last complete sensor disconnection
    std::atomic<bool> m_sensor_disconnected{false}; // Flag indicating sensor is completely disconnected

    IMUData m_last_data;
    int m_unchanged_data_count;
    std::vector<float> m_calib_gx_samples;
    std::vector<float> m_calib_gy_samples;
    std::vector<float> m_calib_gz_samples;

    // Helper methods for converting config values to MPU6050 driver enums
    MPU6050AccelConfig mapAccelConfig(int config_value) const;
    MPU6050GyroConfig mapGyroConfig(int config_value) const;

    static constexpr uint8_t I2C_FAILURE_THRESHOLD = 5; // Consecutive I2C errors before reporting
    static constexpr uint8_t NO_DATA_FAILURE_THRESHOLD = 5; // Consecutive 'responsive but no data' checks before triggering error
    static constexpr int64_t IMU_DATA_TIMEOUT_US = 500000; // 500ms timeout
    static constexpr int64_t PROACTIVE_CHECK_INTERVAL_US = 10 * 1000 * 1000; // 10 seconds
    static constexpr int STUCK_DATA_THRESHOLD = 20;
    static constexpr float MAX_GYRO_RATE_DPS = 2000.0f;
    static constexpr float MAX_ACCEL_G = 16.0f;

};