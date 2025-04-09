// ================================================
// File: main/services/include/IMUService.hpp
// ================================================
#pragma once

#include "mpu6050.hpp"
#include "ConfigData.hpp"
#include "EventBus.hpp"
#include "OrientationEstimator.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <atomic>
#include <vector> // For calibration storage

// Forward declarations
class BaseEvent;
class StartCalibrationRequestEvent;
class CalibrationCompleteEvent;

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

private:
    static constexpr const char* TAG = "IMUService";

    const MPU6050Config m_config;
    EventBus& m_eventBus;
    OrientationEstimator& m_estimator;
    MPU6050 m_sensor;

    TaskHandle_t m_fifo_task_handle;
    SemaphoreHandle_t m_calibration_mutex;
    volatile bool m_is_calibrating; // Should be accessed carefully across tasks/ISRs

    // Store offsets directly in DPS (Degrees Per Second)
    float m_gyro_offset_dps[3]; // Index 0:X, 1:Y, 2:Z

    std::atomic<uint8_t> m_isr_data_counter; // Counter for FIFO samples read by ISR

    // Failure Tracking
    static const uint8_t I2C_FAILURE_THRESHOLD = 5; // Consecutive I2C errors before reporting
    uint8_t m_consecutive_i2c_failures = 0;

    // Event Handling (Example, could be lambda in init)
    void handleStartCalibrationRequest(const BaseEvent& event);

    // Internal Methods
    // Static ISR handler (needs IRAM_ATTR on definition)
    static void IRAM_ATTR isrHandler(void* arg);
    // Member function called by static handler (no IRAM_ATTR needed here)
    void handleInterrupt();

    static void fifo_task_wrapper(void *arg);
    void fifo_task();
    esp_err_t performCalibration();

    // Calibration storage vectors (example for stdev calculation)
    std::vector<float> m_calib_gx_samples;
    std::vector<float> m_calib_gy_samples;
    std::vector<float> m_calib_gz_samples;
};