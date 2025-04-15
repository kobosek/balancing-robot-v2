// main/state/include/StateManager.hpp
#pragma once

#include "SystemState.hpp"
#include "EventBus.hpp"
#include <mutex>
#include <string>
#include "esp_log.h"
#include "esp_timer.h"

// Forward declarations
class BaseEvent;
class FallDetectionEvent;
class StartBalancingCommand;
class StopCommand;
class BatteryStatusUpdatedEvent;
class OrientationDataEvent;
class CalibrateCommandEvent; 
class CalibrationCompleteEvent; 
class EnableRecoveryCommandEvent; 
class DisableRecoveryCommandEvent; 
class EnableFallDetectCommandEvent; 
class DisableFallDetectCommandEvent; 
class IMU_CommunicationErrorEvent; 
class ImuRecoverySucceededEvent; 
class ImuRecoveryFailedEvent; 

class StateManager {
public:
    StateManager(EventBus& eventBus); 
    ~StateManager() = default;

    SystemState getCurrentState() const;
    void setState(SystemState newState);
    esp_err_t init(); 

    // Method for enabling/disabling auto recovery
    void setAutoRecovery(bool enabled);
    bool isAutoRecoveryEnabled() const;
    void enableFallDetection(bool enabled); 
    bool isFallDetectionEnabled() const;    

private:
    static constexpr const char* TAG = "StateManager";
    static constexpr float RECOVERY_ANGLE_THRESHOLD_RAD = 0.087f; 
    static constexpr uint64_t RECOVERY_HOLD_TIME_US = 2000000; 
    // IMU Recovery constants
    static const uint8_t MAX_IMU_RECOVERY_ATTEMPTS = 3;
    static const uint32_t IMU_RECOVERY_DELAY_MS = 1000; 

    EventBus& m_eventBus;
    SystemState m_currentState;

    // State for recovery logic
    bool m_withinRecoveryAngle = false;
    int64_t m_recoveryAngleStartTimeUs = 0;
    bool m_autoRecoveryEnabled = true; 
    bool m_fallDetectionEnabled = true; 
    // State for IMU recovery logic
    SystemState m_preImuRecoveryState = SystemState::IDLE;
    uint8_t m_imu_recovery_attempts = 0;

    // Event handler methods (private)
    void handleFallDetected(const FallDetectionEvent& event);
    void handleStartBalancing(const StartBalancingCommand& event);
    void handleStop(const StopCommand& event);
    void handleBatteryUpdate(const BatteryStatusUpdatedEvent& event);
    void handleOrientationUpdate(const OrientationDataEvent& event);
    void handleCalibrateCommand(const CalibrateCommandEvent& event); 
    void handleCalibrationComplete(const CalibrationCompleteEvent& event); 
    void handleEnableRecovery(const EnableRecoveryCommandEvent& event); 
    void handleDisableRecovery(const DisableRecoveryCommandEvent& event); 
    void handleEnableFallDetect(const EnableFallDetectCommandEvent& event);  
    void handleDisableFallDetect(const DisableFallDetectCommandEvent& event); 
    void handleImuCommunicationError(const IMU_CommunicationErrorEvent& event); 
    void handleImuRecoverySucceeded(const ImuRecoverySucceededEvent& event); 
    void handleImuRecoveryFailed(const ImuRecoveryFailedEvent& event); 

    // Recovery helpers
    void requestImuRecovery(); 
    void handleRecoveryFailure(); 

    // Helper for logging state names
    std::string stateToString(SystemState state) const;
};