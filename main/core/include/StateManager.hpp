// ================================================
// File: main/core/include/StateManager.hpp
// ================================================
#pragma once

#include "SystemState.hpp"
#include "EventBus.hpp"
#include "EventHandler.hpp"
#include <mutex>
#include <string>
#include "esp_log.h"
#include "esp_timer.h"

// Forward declarations
class BaseEvent;
class BALANCE_FallDetected;
class BALANCE_RecoveryDetected;
class UI_StartBalancing;
class UI_Stop;
class BATTERY_StatusUpdate;
class UI_CalibrateImu;
class IMU_CalibrationCompleted;
class CONFIG_FullConfigUpdate;
class IMU_CalibrationRequest;
class IMU_CalibrationRequestRejected;
class IMU_CommunicationError;
class IMU_AvailabilityChanged;

struct SystemBehaviorConfig; 
struct ConfigData; 

class StateManager : public EventHandler {
public:
    StateManager(EventBus& eventBus, const SystemBehaviorConfig& initialBehaviorConfig);
    ~StateManager() = default;

    SystemState getCurrentState() const;
    void setState(SystemState newState);

    esp_err_t init();
    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }
    
    // Kept for backward compatibility
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "StateManager";

    EventBus& m_eventBus;
    SystemState m_currentState;
    bool m_imu_available = false;
    bool m_pending_calibration = false;
    bool m_pending_start = false;

    void handleFallDetected(const BALANCE_FallDetected& event);
    void handleRecoveryDetected(const BALANCE_RecoveryDetected& event);
    void handleStartBalancing(const UI_StartBalancing& event);
    void handleStop(const UI_Stop& event);
    void handleBatteryUpdate(const BATTERY_StatusUpdate& event);
    void handleCalibrateCommand(const UI_CalibrateImu& event);
    void handleCalibrationComplete(const IMU_CalibrationCompleted& event);
    void handleImuCommunicationError(const IMU_CommunicationError& event);
    void handleImuAvailabilityChanged(const IMU_AvailabilityChanged& event);
    void handleCalibrationRejected(const IMU_CalibrationRequestRejected& event);
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);

    void initiateCalibration(bool force = false);
    
    std::string stateToString(SystemState state) const;
    void applyConfig(const SystemBehaviorConfig& config);
};
