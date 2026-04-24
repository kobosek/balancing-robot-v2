// ================================================
// File: main/core/include/StateManager.hpp
// ================================================
#pragma once

#include "EventBus.hpp"
#include "EventHandler.hpp"
#include <mutex>
#include <string>
#include "esp_log.h"
#include "esp_timer.h"

// Forward declarations
class BaseEvent;
class BALANCE_FallDetected;
class BALANCE_AutoBalanceReady;
class UI_StartBalancing;
class UI_Stop;
class UI_EnableAutoBalancing;
class UI_DisableAutoBalancing;
class UI_EnableFallDetection;
class UI_DisableFallDetection;
class BATTERY_StatusUpdate;
class UI_CalibrateImu;
class UI_StartPidTuning;
class UI_CancelPidTuning;
class UI_StartGuidedCalibration;
class UI_CancelGuidedCalibration;
class IMU_CalibrationCompleted;
class PID_TuningFinished;
class GUIDED_CalibrationFinished;
class CONFIG_FullConfigUpdate;
class IMU_CalibrationRequest;
class IMU_CalibrationRequestRejected;
class IMU_CommunicationError;
class IMU_AvailabilityChanged;
enum class SystemState;

struct SystemBehaviorConfig; 
struct BatteryConfig;
struct ConfigData; 

struct SystemStatusSnapshot {
    int stateId;
    const char* stateName;
    bool autoBalancingEnabled;
    bool fallDetectionEnabled;
    bool criticalBatteryMotorShutdownEnabled;
};

class StateManager : public EventHandler {
public:
    StateManager(EventBus& eventBus, const SystemBehaviorConfig& initialBehaviorConfig, const BatteryConfig& initialBatteryConfig);
    ~StateManager() = default;

    SystemStatusSnapshot getStatusSnapshot() const;
    bool isCriticalBatteryMotorShutdownEnabled() const {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        return m_criticalBatteryMotorShutdownEnabled;
    }
    bool isAutoBalancingEnabled() const {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        return m_autoBalancingEnabled;
    }
    bool isFallDetectionEnabled() const {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        return m_fallDetectionEnabled;
    }
    void markReady();
    void markFatalError();

    esp_err_t init();
    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }
    
    // Kept for backward compatibility
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "StateManager";

    EventBus& m_eventBus;
    mutable std::recursive_mutex m_mutex;
    SystemState m_currentState;
    bool m_imu_available = false;
    bool m_pending_calibration = false;
    bool m_pending_start = false;
    bool m_battery_critical = false;
    bool m_criticalBatteryMotorShutdownEnabled = false;
    bool m_autoBalancingEnabled = true;
    bool m_fallDetectionEnabled = true;

    void setState(SystemState newState);

    void handleFallDetected(const BALANCE_FallDetected& event);
    void handleAutoBalanceReady(const BALANCE_AutoBalanceReady& event);
    void handleStartBalancing(const UI_StartBalancing& event);
    void handleStop(const UI_Stop& event);
    void handleEnableAutoBalancing(const UI_EnableAutoBalancing& event);
    void handleDisableAutoBalancing(const UI_DisableAutoBalancing& event);
    void handleEnableFallDetect(const UI_EnableFallDetection& event);
    void handleDisableFallDetect(const UI_DisableFallDetection& event);
    void handleBatteryUpdate(const BATTERY_StatusUpdate& event);
    void handleCalibrateCommand(const UI_CalibrateImu& event);
    void handleStartPidTuning(const UI_StartPidTuning& event);
    void handleCancelPidTuning(const UI_CancelPidTuning& event);
    void handlePidTuningFinished(const PID_TuningFinished& event);
    void handleStartGuidedCalibration(const UI_StartGuidedCalibration& event);
    void handleCancelGuidedCalibration(const UI_CancelGuidedCalibration& event);
    void handleGuidedCalibrationFinished(const GUIDED_CalibrationFinished& event);
    void handleCalibrationComplete(const IMU_CalibrationCompleted& event);
    void handleImuCommunicationError(const IMU_CommunicationError& event);
    void handleImuAvailabilityChanged(const IMU_AvailabilityChanged& event);
    void handleCalibrationRejected(const IMU_CalibrationRequestRejected& event);
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);

    void initiateCalibration(bool force = false);
    void publishStateDerivedModes();
    void publishBalanceMonitorMode();
    void publishMotorOutputMode();
    void publishRoutineRunModes();
    void publishCommandInputMode();
    void publishControlRunMode();
    void publishImuSystemPolicy();
    void publishOtaUpdatePolicy();

    void applyConfig(const SystemBehaviorConfig& behaviorConfig, const BatteryConfig& batteryConfig);
};
