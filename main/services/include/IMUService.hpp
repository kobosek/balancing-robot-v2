// main/services/include/IMUService.hpp
#pragma once

#include "mpu6050.hpp"
#include "ConfigData.hpp"
#include "EventBus.hpp"
#include "OrientationEstimator.hpp" // Keep include
#include "freertos/FreeRTOS.h" // For BaseType_t if needed in header, maybe not
#include "freertos/task.h" // For TaskHandle_t
#include "freertos/semphr.h" // For SemaphoreHandle_t
// #include "esp_log.h" // Move to cpp if possible
#include <atomic>
#include <memory> // Not needed for fwd decls

// Forward declare events handled/published
class BaseEvent;
class StartCalibrationRequestEvent;
class CalibrationCompleteEvent;

class IMUService {
public:
    IMUService(const MPU6050Config& config, EventBus& bus, OrientationEstimator& estimator);
    ~IMUService();

    esp_err_t init();
    esp_err_t triggerCalibration(); // This can now be called by event handler

    // --- Methods for Recovery State ---
    esp_err_t resetSensor();
    esp_err_t reinitializeSensor();
    esp_err_t verifyCommunication();

private:
    static constexpr const char* TAG = "IMUService";

    const MPU6050Config m_config;
    EventBus& m_eventBus;
    OrientationEstimator& m_estimator;
    MPU6050 m_sensor;

    TaskHandle_t m_fifo_task_handle; // Declaration
    SemaphoreHandle_t m_calibration_mutex; // Declaration
    volatile bool m_is_calibrating; // Declaration
    float m_gyro_offset_radps[3]; // Declaration

    std::atomic<uint8_t> m_isr_data_counter; // Declaration

    // --- Failure Tracking ---
    static const uint8_t I2C_FAILURE_THRESHOLD = 5; // Max consecutive failures before reporting
    uint8_t m_consecutive_i2c_failures = 0;


    // --- Event Handling ---
    void handleStartCalibrationRequest(const BaseEvent& event);

    // --- Existing methods ---
    // Static ISR handler needs IRAM_ATTR on definition ONLY
    static void IRAM_ATTR isrHandler(void* arg);
    // Member function called by static handler also needs IRAM_ATTR on definition ONLY
    void handleInterrupt(); // REMOVED IRAM_ATTR from declaration

    static void fifo_task_wrapper(void *arg);
    void fifo_task();
    esp_err_t performCalibration();
};