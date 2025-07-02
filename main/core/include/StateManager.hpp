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
class MOTION_FallDetected;
class UI_StartBalancing;
class UI_Stop;
class BATTERY_StatusUpdate;
class IMU_OrientationData;
class UI_CalibrateImu;
class IMU_CalibrationCompleted;
class UI_EnableFallRecovery;
class UI_DisableFallRecovery;
class UI_EnableFallDetection;
class UI_DisableFallDetection;
class IMU_RecoverySucceeded;
class IMU_RecoveryFailed;
class CONFIG_FullConfigUpdate; 
class IMU_CalibrationRequest;
class IMU_CalibrationRequestRejected;

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

    void setAutoRecovery(bool enabled);
    bool isAutoRecoveryEnabled() const;
    void enableFallDetection(bool enabled);
    bool isFallDetectionEnabled() const;

private:
    static constexpr const char* TAG = "StateManager";

    EventBus& m_eventBus;
    SystemState m_currentState;

    bool m_withinRecoveryAngle = false;
    int64_t m_recoveryAngleStartTimeUs = 0;
    bool m_autoRecoveryEnabled = true;
    bool m_fallDetectionEnabled = true;
    SystemState m_preImuRecoveryState = SystemState::IDLE;
    uint8_t m_imu_recovery_attempts = 0;
    bool m_pending_calibration = false; 

    float m_recovery_angle_threshold_rad;
    uint64_t m_recovery_hold_time_us;
    uint8_t m_max_imu_recovery_attempts;

    void handleFallDetected(const MOTION_FallDetected& event);
    void handleStartBalancing(const UI_StartBalancing& event);
    void handleStop(const UI_Stop& event);
    void handleBatteryUpdate(const BATTERY_StatusUpdate& event);
    void handleOrientationUpdate(const IMU_OrientationData& event);
    void handleCalibrateCommand(const UI_CalibrateImu& event);
    void handleCalibrationComplete(const IMU_CalibrationCompleted& event);
    void handleEnableRecovery(const UI_EnableFallRecovery& event);
    void handleDisableRecovery(const UI_DisableFallRecovery& event);
    void handleEnableFallDetect(const UI_EnableFallDetection& event);
    void handleDisableFallDetect(const UI_DisableFallDetection& event);
    void handleImuRecoverySucceeded(const IMU_RecoverySucceeded& event);
    void handleImuRecoveryFailed(const IMU_RecoveryFailed& event);
    void handleCalibrationRejected(const IMU_CalibrationRequestRejected& event);
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);

    void initiateCalibration(bool force = false);
    
    std::string stateToString(SystemState state) const;
    void applyConfig(const SystemBehaviorConfig& config);
};