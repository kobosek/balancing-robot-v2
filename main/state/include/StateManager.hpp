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
class CalibrateCommandEvent; // <<< ADDED
class CalibrationCompleteEvent; // <<< ADDED
class EnableRecoveryCommandEvent; // <<< ADDED
class DisableRecoveryCommandEvent; // <<< ADDED
class EnableFallDetectCommandEvent; // <<< ADDED
class DisableFallDetectCommandEvent; // <<< ADDED

class StateManager {
public:
    StateManager(EventBus& eventBus);
    ~StateManager() = default;

    SystemState getCurrentState() const;
    void setState(SystemState newState);
    esp_err_t init(); // Subscribe to events

    // Method for enabling/disabling auto recovery
    void setAutoRecovery(bool enabled);
    bool isAutoRecoveryEnabled() const;
    void enableFallDetection(bool enabled); // <<< ADDED
    bool isFallDetectionEnabled() const;    // <<< ADDED

private:
    static constexpr const char* TAG = "StateManager";
    static constexpr float RECOVERY_ANGLE_THRESHOLD_RAD = 0.087f; // Approx 5 degrees
    static constexpr uint64_t RECOVERY_HOLD_TIME_US = 2000000; // 2 seconds <<< UPDATED

    EventBus& m_eventBus;
    SystemState m_currentState;

    // State for recovery logic
    bool m_withinRecoveryAngle = false;
    int64_t m_recoveryAngleStartTimeUs = 0;
    bool m_autoRecoveryEnabled = true; // <<< ADDED - Default to enabled
    bool m_fallDetectionEnabled = true; // <<< ADDED (Default true)

    // Event handler methods (private)
    void handleFallDetected(const FallDetectionEvent& event);
    void handleStartBalancing(const StartBalancingCommand& event);
    void handleStop(const StopCommand& event);
    void handleBatteryUpdate(const BatteryStatusUpdatedEvent& event);
    void handleOrientationUpdate(const OrientationDataEvent& event);
    void handleCalibrateCommand(const CalibrateCommandEvent& event); // <<< ADDED
    void handleCalibrationComplete(const CalibrationCompleteEvent& event); // <<< ADDED
    void handleEnableRecovery(const EnableRecoveryCommandEvent& event); // <<< ADDED
    void handleDisableRecovery(const DisableRecoveryCommandEvent& event); // <<< ADDED
    void handleEnableFallDetect(const EnableFallDetectCommandEvent& event);  // <<< ADDED
    void handleDisableFallDetect(const DisableFallDetectCommandEvent& event); // <<< ADDED

    // Helper for logging state names
    std::string stateToString(SystemState state) const;
};