#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <memory> // For unique_ptr if owned components

// Forward declarations
class MPU6050Driver;
class OrientationEstimator;
class ConfigService; // Keep ConfigService for overall config
class EventBus;
class BaseEvent;
class IMUCalibrationService; // Use Interface
class IMUHealthMonitor; // <<< Already Forward Declared
class IMUHealthMonitor;      // Use Interface
struct MPU6050Config;
class IMU_CommunicationErrorEvent; // For subscription
class AttemptImuRecoveryCommand; // For subscription


class IMUService {
public:
    // Constructor takes dependencies
    IMUService(MPU6050Driver& driver,
               IMUCalibrationService& calibrationService,
               IMUHealthMonitor& healthMonitor,
               OrientationEstimator& estimator, // Pass estimator by reference
               const MPU6050Config& config, // Still need config details
               EventBus& bus);
    ~IMUService();

    esp_err_t init();

    // Core data processing function (called by task)
    void processDataPipeline();

    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "IMUService";

    // References to components (passed in constructor)
    MPU6050Driver& m_driver;
    IMUCalibrationService& m_calibrationService;
    IMUHealthMonitor& m_healthMonitor;
    OrientationEstimator& m_estimator;
    const MPU6050Config& m_config; // Keep reference to config
    EventBus& m_eventBus;

    // Stored scaling factors (calculated in init)
    float m_accel_lsb_per_g;
    float m_gyro_lsb_per_dps;
    float m_sample_period_s;

    // FIFO processing state
    static constexpr size_t FIFO_PACKET_SIZE = 12;
    static constexpr size_t MAX_FIFO_BUFFER_SIZE = 1024; // Max HW FIFO size
    uint8_t m_fifo_buffer[MAX_FIFO_BUFFER_SIZE];

    std::atomic<uint8_t> m_isr_data_counter; // ISR still increments this

    // Recovery state
    std::atomic<bool> m_recovery_in_progress{false};
    static const uint8_t MAX_IMU_RECOVERY_ATTEMPTS = 3;
    uint8_t m_imu_recovery_attempts = 0;

    // --- Private Methods ---
    static void IRAM_ATTR isrHandler(void* arg);
    esp_err_t configureSensorHardware();
    esp_err_t attemptRecovery();
    void calculateScalingFactors();

    // Event Handlers (called via lambda subscription)
    void handleImuCommunicationError(const IMU_CommunicationErrorEvent& event);
    // void handleAttemptRecoveryCommand(const AttemptImuRecoveryCommand& event); // Can be handled by comm error
};