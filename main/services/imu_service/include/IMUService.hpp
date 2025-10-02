// main/services/include/IMUService.hpp
#pragma once

#include "ConfigData.hpp"
#include "SystemState.hpp"
#include "IMUState.hpp"
#include "EventHandler.hpp"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <functional> 

// Forward declarations
class MPU6050Driver;
class OrientationEstimator;
class EventBus;
class BaseEvent;
class IMUCalibration;
class IMU_CommunicationError;
class IMU_AttemptRecovery; 
class CONFIG_FullConfigUpdate; 
class CONFIG_ImuConfigUpdate; 
class IMU_CalibrationCompleted; 
class IMU_CalibrationRequest;
class SYSTEM_StateChanged;
class IMU_StateTransitionRequest;

// Task forward declarations
class Task;
class FIFOTask;
class HealthMonitorTask;

// Forward declare IMU components that will be encapsulated
class IMUHealthMonitor;
class FIFOProcessor;

class IMUService : public EventHandler {
public:
    // Updated constructor that doesn't take references to components we'll create internally
    IMUService(std::shared_ptr<OrientationEstimator> estimator,
               const MPU6050Config& config,
               const SystemBehaviorConfig& behaviorConfig,
               EventBus& bus);
    ~IMUService();

    esp_err_t init();
    
    // Start the IMU-related tasks
    bool startTasks();
    
    // Stop the IMU-related tasks
    void stopTasks();
    void processDataPipeline();
    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }
    
    IMUState getCurrentState() const;
    SystemState getSystemState() const;
    void updateSystemState(SystemState newState);

    // Made public static for use by other services if they log states
    static const char* stateToString(IMUState state);

private:
    static constexpr const char* TAG = "IMUService";

    std::unique_ptr<MPU6050Driver> m_driver;
    std::shared_ptr<OrientationEstimator> m_estimator;
    MPU6050Config m_config;
    SystemBehaviorConfig m_behaviorConfig;
    EventBus& m_eventBus;
    
    // Owned components
    std::unique_ptr<IMUHealthMonitor> m_healthMonitor;
    std::unique_ptr<FIFOProcessor> m_fifoProcessor;
    std::unique_ptr<IMUCalibration> m_calibration;
    
    // Task components
    std::unique_ptr<FIFOTask> m_fifoTask;
    std::unique_ptr<HealthMonitorTask> m_healthMonitorTask;
    std::atomic<bool> m_is_calibrating_flag;  // Tracks if calibration is in progress

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

    // Thread safety
    mutable std::mutex m_state_mutex;           // Protects state variables and configuration
    mutable std::mutex m_config_mutex;          // Protects configuration variables
    static constexpr uint32_t MUTEX_TIMEOUT_MS = 100;  // Timeout for mutex operations

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
    
    // FIFO management now delegated to FIFOProcessor

    // Private Methods
    esp_err_t configureSensorHardware(); // Applies m_config to hardware
    esp_err_t attemptRecovery(const IMURecoveryConfig& config); // Low-level recovery implementation
    void startRecoveryProcess(const IMURecoveryConfig& config); // High-level recovery coordinator
    void calculateScalingFactors(); 
    void applyConfig(const MPU6050Config& newConfig);
    void applyConfig(const SystemBehaviorConfig& config);
    
    // Event Handlers
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);
    void handleIMUConfigUpdate(const CONFIG_ImuConfigUpdate& event);
    void handleImuCommunicationError(const IMU_CommunicationError& event);
    void handleCalibrationRequest(const IMU_CalibrationRequest& event);

    void handleSystemStateChanged(const SYSTEM_StateChanged& event);
    
    // Calibration method called directly within IMUService
    esp_err_t performCalibration();

    // Thread-safe helper methods
    bool tryLockWithTimeout(std::mutex& mutex, uint32_t timeout_ms) const;
    void safeConfigUpdate(const std::function<void()>& updateFunc);
    std::pair<IMUState, SystemState> getStatesThreadSafe() const;
};