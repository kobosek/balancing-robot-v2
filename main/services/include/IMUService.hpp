// main/services/include/IMUService.hpp
#pragma once

#include "ConfigData.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <memory> // For unique_ptr if owned components

// Forward declarations
class MPU6050Driver;
class OrientationEstimator;
class EventBus;
class BaseEvent;
class IMUCalibrationService;
class IMUHealthMonitor;
class IMU_CommunicationErrorEvent;
class AttemptImuRecoveryCommand; // Correct forward declaration for subscription
class ConfigUpdatedEvent; // Need event definition

class IMUService {
public:
    // Constructor takes dependencies
    IMUService(MPU6050Driver& driver,
               IMUCalibrationService& calibrationService,
               IMUHealthMonitor& healthMonitor,
               OrientationEstimator& estimator,
               const MPU6050Config& config, // Still need initial config details
               EventBus& bus);
    ~IMUService();

    esp_err_t init();

    // Core data processing function (called by task)
    void processDataPipeline();

    // Subscribe to events
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "IMUService";

    // References to components (passed in constructor)
    MPU6050Driver& m_driver;
    IMUCalibrationService& m_calibrationService;
    IMUHealthMonitor& m_healthMonitor;
    OrientationEstimator& m_estimator;
    MPU6050Config m_config; // Store a copy of the config
    EventBus& m_eventBus;

    // Stored scaling factors (calculated in init/on update)
    float m_accel_lsb_per_g;
    float m_gyro_lsb_per_dps;
    float m_sample_period_s;

    // --- Constants ---
    static constexpr size_t FIFO_PACKET_SIZE = 12; // Size of one Accel+Gyro sample
    static constexpr size_t MAX_FIFO_BUFFER_SIZE = 1024; // Max bytes we read into our buffer
    // <<< NEW: Max samples to process from FIFO in one go >>>
    static constexpr uint16_t MAX_SAMPLES_PER_PIPELINE_CALL = 20;

    // FIFO processing state
    uint8_t m_fifo_buffer[MAX_FIFO_BUFFER_SIZE];
    std::atomic<uint8_t> m_isr_data_counter;

    // Recovery state
    std::atomic<bool> m_recovery_in_progress{false};
    // uint8_t m_imu_recovery_attempts = 0; // <<< REMOVED >>>

    // <<< ADDED: Track if ISR handler was successfully installed >>>
    bool m_isr_handler_installed = false;

    // --- Private Methods ---
    static void IRAM_ATTR isrHandler(void* arg);
    esp_err_t configureSensorHardware();
    esp_err_t attemptRecovery(); // Now includes immediate retries
    void calculateScalingFactors(); // Takes config from member variable
    void applyConfig(const MPU6050Config& newConfig); // Apply new config values
    void handleConfigUpdate(const BaseEvent& event); // Handle config update event

    // Event Handlers (called via lambda subscription)
    void handleImuCommunicationError(const IMU_CommunicationErrorEvent& event);
    // void handleAttemptRecoveryCommand(const BaseEvent& event); // Handled via lambda now
};