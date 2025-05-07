// main/services/include/IMUService.hpp
#pragma once

#include "ConfigData.hpp"
#include "SystemState.hpp"
#include "IMUState.hpp"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <memory> 

// Forward declarations
class MPU6050Driver;
class OrientationEstimator;
class EventBus;
class BaseEvent;
class IMUCalibrationService;
class IMUHealthMonitor;
class IMU_CommunicationError;
class IMU_AttemptRecovery; 
class CONFIG_FullConfigUpdate; 
class CONFIG_ImuConfigUpdate; 
class IMU_CalibrationStarted; 
class IMU_CalibrationCompleted; 


class IMUService {
public:
    IMUService(MPU6050Driver& driver,
               IMUCalibrationService& calibrationService,
               IMUHealthMonitor& healthMonitor,
               OrientationEstimator& estimator,
               const MPU6050Config& config, 
               EventBus& bus);
    ~IMUService();

    esp_err_t init();
    void processDataPipeline();
    void subscribeToEvents(EventBus& bus);
    
    IMUState getCurrentState() const { return m_current_state.load(std::memory_order_relaxed); }
    SystemState getSystemState() const { return m_system_state.load(std::memory_order_relaxed); }
    void updateSystemState(SystemState newState) { m_system_state.store(newState, std::memory_order_relaxed); }

    // Made public static for use by other services if they log states
    static const char* stateToString(IMUState state);

private:
    static constexpr const char* TAG = "IMUService";

    MPU6050Driver& m_driver;
    IMUCalibrationService& m_calibrationService;
    IMUHealthMonitor& m_healthMonitor;
    OrientationEstimator& m_estimator;
    MPU6050Config m_config; 
    EventBus& m_eventBus;

    std::atomic<IMUState> m_current_state; // Default initialized by constructor if needed
    std::atomic<SystemState> m_system_state;

    // Recovery tracking
    std::atomic<bool> m_recovery_attempt_in_progress;
    // esp_timer_handle_t m_recovery_watchdog_timer; // Unused, removed
    uint32_t m_recovery_start_time_ms;
    uint32_t m_recovery_timeout_ms; // Default timeout for a recovery attempt

    float m_accel_lsb_per_g;
    float m_gyro_lsb_per_dps;
    float m_sample_period_s;

    static constexpr size_t FIFO_PACKET_SIZE = 12; 
    static constexpr size_t MAX_FIFO_BUFFER_SIZE = 1024; 
    static constexpr uint16_t MAX_SAMPLES_PER_PIPELINE_CALL = 20; // Max MPU samples (6 floats) to process per task cycle

    uint8_t m_fifo_buffer[MAX_FIFO_BUFFER_SIZE];
    std::atomic<uint8_t> m_isr_data_counter; // Incremented by ISR, read and reset by task

    bool m_isr_handler_installed;
    
    // State Machine Methods
    void transitionToState(IMUState newState);
    esp_err_t enterInitializedState();
    esp_err_t exitInitializedState();
    esp_err_t enterOperationalState();
    esp_err_t exitOperationalState();
    esp_err_t enterCalibrationState();
    esp_err_t exitCalibrationState();
    esp_err_t enterRecoveryState();
    esp_err_t exitRecoveryState();
    
    static bool isValidTransition(IMUState from, IMUState to);
    
    // FIFO management helpers
    esp_err_t enableFIFO();
    esp_err_t disableFIFO();
    esp_err_t resetFIFO();
    esp_err_t resetAndReEnableFIFO(); // Composite helper
    
    bool validateFIFOData(const uint8_t* data, size_t length);

    // Private Methods
    static void IRAM_ATTR isrHandler(void* arg);
    esp_err_t configureSensorHardware(); // Applies m_config to hardware
    esp_err_t attemptRecovery(const IMURecoveryConfig& config); // Performs recovery steps
    void calculateScalingFactors(); 
    void applyConfig(const MPU6050Config& newConfig);

    // Event Handlers
    void handleConfigUpdate(const BaseEvent& event); // For CONFIG_FullConfigUpdate
    void handleIMUConfigUpdate(const CONFIG_ImuConfigUpdate& event);
    void handleImuCommunicationError(const IMU_CommunicationError& event);
    void handleCalibrationStarted(const IMU_CalibrationStarted& event);
    void handleCalibrationComplete(const IMU_CalibrationCompleted& event);
    void handleAttemptImuRecovery(const IMU_AttemptRecovery& event);
    void handleSystemStateChanged(const BaseEvent& event); // For SYSTEM_StateChanged
};