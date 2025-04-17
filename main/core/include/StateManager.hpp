// ================================================
// File: main/core/include/StateManager.hpp
// ================================================
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
class AttemptImuRecoveryCommand;
class ConfigUpdatedEvent; // Need event definition
// class ConfigurationService; // No longer needed
struct SystemBehaviorConfig; // Forward declare needed config struct
struct ConfigData; // Needed by handler

class StateManager {
public:
    // Constructor now takes initial config struct
    StateManager(EventBus& eventBus, const SystemBehaviorConfig& initialBehaviorConfig);
    ~StateManager() = default;

    SystemState getCurrentState() const;
    void setState(SystemState newState);

    esp_err_t init();
    void subscribeToEvents(EventBus& bus);

    void setAutoRecovery(bool enabled);
    bool isAutoRecoveryEnabled() const;
    void enableFallDetection(bool enabled);
    bool isFallDetectionEnabled() const;

private:
    static constexpr const char* TAG = "StateManager";

    EventBus& m_eventBus;
    // ConfigurationService& m_configService; // REMOVE
    SystemState m_currentState;

    // State for recovery logic
    bool m_withinRecoveryAngle = false;
    int64_t m_recoveryAngleStartTimeUs = 0;
    bool m_autoRecoveryEnabled = true;
    bool m_fallDetectionEnabled = true;
    // State for IMU recovery logic
    SystemState m_preImuRecoveryState = SystemState::IDLE;
    uint8_t m_imu_recovery_attempts = 0;

    // Local copies of configuration values for performance/convenience
    float m_recovery_angle_threshold_rad;
    uint64_t m_recovery_hold_time_us;
    uint8_t m_max_imu_recovery_attempts;
    // uint32_t m_imu_recovery_delay_ms; // Not currently used directly

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
    void handleConfigUpdate(const BaseEvent& event); // Add event handler

    // Recovery helpers
    void requestImuRecovery();
    void handleRecoveryFailure();

    // Helper for logging state names
    std::string stateToString(SystemState state) const;
    // Helper to apply config values
    void applyConfig(const SystemBehaviorConfig& config);
};