// ================================================
// File: main/core/StateManager.cpp
// ================================================
#include "StateManager.hpp"

// Include all necessary event headers
#include "SYSTEM_StateChanged.hpp"
#include "BALANCE_FallDetected.hpp"
#include "BALANCE_RecoveryDetected.hpp"
#include "UI_StartBalancing.hpp"
#include "UI_Stop.hpp"
#include "BATTERY_StatusUpdate.hpp"
#include "UI_CalibrateImu.hpp"
#include "IMU_CalibrationRequest.hpp"
#include "IMU_CalibrationCompleted.hpp"
#include "UI_EnableFallRecovery.hpp"
#include "UI_DisableFallRecovery.hpp"
#include "UI_EnableFallDetection.hpp"
#include "UI_DisableFallDetection.hpp"
#include "IMU_CommunicationError.hpp"
#include "CONFIG_FullConfigUpdate.hpp"
#include "IMU_CalibrationRequestRejected.hpp"
#include "IMU_RecoveryFailed.hpp"
#include "IMU_RecoverySucceeded.hpp"

// Other includes
#include "EventTypes.hpp"
#include "BaseEvent.hpp"
#include "EventBus.hpp"
#include "SystemState.hpp"
#include "ConfigData.hpp"
#include <string>
#include "esp_log.h"
#include "esp_err.h"

StateManager::StateManager(EventBus& eventBus, const SystemBehaviorConfig& initialBehaviorConfig) :
    m_eventBus(eventBus),
    m_currentState(SystemState::INIT),
    m_preImuRecoveryState(SystemState::IDLE),
    m_imu_recovery_attempts(0),
    m_max_imu_recovery_attempts(3)
{
    applyConfig(initialBehaviorConfig);
}

esp_err_t StateManager::init() {
    ESP_LOGI(TAG, "Initializing StateManager...");
    ESP_LOGI(TAG, "StateManager Initialized.");
    return ESP_OK;
}

void StateManager::applyConfig(const SystemBehaviorConfig& config) {
    m_max_imu_recovery_attempts = config.imu_recovery_max_attempts;
    ESP_LOGI(TAG, "Applied StateManager params: MaxIMURecov=%d", m_max_imu_recovery_attempts);
}

SystemState StateManager::getCurrentState() const {
    return m_currentState;
}

std::string StateManager::stateToString(SystemState state) const {
    switch(state) {
        case SystemState::INIT: return "INIT";
        case SystemState::IDLE: return "IDLE";
        case SystemState::BALANCING: return "BALANCING";
        case SystemState::FALLEN: return "FALLEN";
        case SystemState::SHUTDOWN: return "SHUTDOWN";
        case SystemState::FATAL_ERROR: return "FATAL_ERROR";
        default: return "UNKNOWN";
    }
}

void StateManager::setState(SystemState newState) {
    SystemState previousState = m_currentState;
    bool stateChanged = false;
    if (m_currentState != newState) {
        m_currentState = newState;
        stateChanged = true;
        ESP_LOGI(TAG, "State changed from %d (%s) to %d (%s)",
                 static_cast<int>(previousState), stateToString(previousState).c_str(),
                 static_cast<int>(newState), stateToString(newState).c_str());

        if (previousState != SystemState::IDLE && newState == SystemState::IDLE) {
            ESP_LOGD(TAG, "Entering IDLE state, checking for pending calibration.");
            if (m_pending_calibration) {
                ESP_LOGI(TAG, "Executing pending calibration request.");
                initiateCalibration(true);
            }
        }
    }

    if (stateChanged) {
        SYSTEM_StateChanged event(previousState, newState);
        m_eventBus.publish(event);
    }
}

void StateManager::handleEvent(const BaseEvent& event) {
    switch (event.type) {
        case EventType::BALANCE_FALL_DETECTED:
            handleFallDetected(static_cast<const BALANCE_FallDetected&>(event));
            break;
        case EventType::BALANCE_RECOVERY_DETECTED:
            handleRecoveryDetected(static_cast<const BALANCE_RecoveryDetected&>(event));
            break;
        case EventType::UI_START_BALANCING:
            handleStartBalancing(static_cast<const UI_StartBalancing&>(event));
            break;

        case EventType::UI_STOP:
            handleStop(static_cast<const UI_Stop&>(event));
            break;

        case EventType::BATTERY_STATUS_UPDATE:
            handleBatteryUpdate(static_cast<const BATTERY_StatusUpdate&>(event));
            break;


        case EventType::UI_CALIBRATE_IMU:
            handleCalibrateCommand(static_cast<const UI_CalibrateImu&>(event));
            break;

        case EventType::IMU_CALIBRATION_COMPLETED:
            handleCalibrationComplete(static_cast<const IMU_CalibrationCompleted&>(event));
            break;

        case EventType::IMU_CALIBRATION_REJECTED:
            handleCalibrationRejected(static_cast<const IMU_CalibrationRequestRejected&>(event));
            break;

        case EventType::IMU_RECOVERY_SUCCEEDED:
            handleImuRecoverySucceeded(static_cast<const IMU_RecoverySucceeded&>(event));
            break;

        case EventType::IMU_RECOVERY_FAILED:
            handleImuRecoveryFailed(static_cast<const IMU_RecoveryFailed&>(event));
            break;

        case EventType::CONFIG_FULL_UPDATE:
            handleConfigUpdate(static_cast<const CONFIG_FullConfigUpdate&>(event));
            break;

        default:
            ESP_LOGV(TAG, "%s: Received unhandled event type %d",
                     getHandlerName().c_str(), static_cast<int>(event.type));
            break;
    }
}

void StateManager::subscribeToEvents(EventBus& bus) {
    ESP_LOGW(TAG, "StateManager::subscribeToEvents is deprecated. Use EventBus::subscribe with EventHandler instead.");
}

void StateManager::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
    applyConfig(event.configData.behavior);
}

void StateManager::handleFallDetected(const BALANCE_FallDetected& event) {
    ESP_LOGW(TAG, "Fall Detected Event Received!");
    if (m_currentState == SystemState::BALANCING) {
        ESP_LOGW(TAG, "Transitioning to FALLEN state due to fall detection.");
        setState(SystemState::FALLEN);
    } else {
        ESP_LOGW(TAG, "Fall detected but not in BALANCING state (%s), ignoring.", stateToString(m_currentState).c_str());
    }
}

void StateManager::handleRecoveryDetected(const BALANCE_RecoveryDetected& event) {
    ESP_LOGW(TAG, "Recovery Detected Event Received!");
    if (m_currentState == SystemState::FALLEN) {
        ESP_LOGW(TAG, "Transitioning to Balancing state due to recovery detection.");
        setState(SystemState::BALANCING);
    } else {
        ESP_LOGW(TAG, "Recovery detected but not in FALLEN state (%s), ignoring.", stateToString(m_currentState).c_str());
    }
}

void StateManager::handleStartBalancing(const UI_StartBalancing& event) {
    ESP_LOGI(TAG, "Start Balancing Command Received by StateManager.");
    if (m_currentState == SystemState::IDLE || m_currentState == SystemState::FALLEN) {
        ESP_LOGI(TAG, "Transitioning to BALANCING state.");
        setState(SystemState::BALANCING);
    } else {
        ESP_LOGW(TAG, "Cannot start balancing from current state: %s", stateToString(m_currentState).c_str());
    }
}

void StateManager::handleStop(const UI_Stop& event) {
    ESP_LOGI(TAG, "Stop Command Received by StateManager.");
    if (m_currentState != SystemState::IDLE && m_currentState != SystemState::INIT) {
        ESP_LOGI(TAG, "Transitioning to IDLE state from %s.", stateToString(m_currentState).c_str());
    }
    setState(SystemState::IDLE);
}

void StateManager::handleBatteryUpdate(const BATTERY_StatusUpdate& event) {
    ESP_LOGV(TAG, "Battery Update Received: %.2fV, %d%%", event.status.voltage, event.status.percentage);
    if (event.status.isLow && m_currentState == SystemState::BALANCING) {
        ESP_LOGW(TAG, "Battery level LOW while balancing! Transitioning to IDLE.");
        setState(SystemState::IDLE);
    }
}

void StateManager::handleCalibrateCommand(const UI_CalibrateImu& event) {
    ESP_LOGI(TAG, "Calibrate Command Received.");
    initiateCalibration(false);
}

void StateManager::handleCalibrationComplete(const IMU_CalibrationCompleted& event) {
    ESP_LOGI(TAG, "Calibration Complete Event Received (Status: %s).", esp_err_to_name(event.status));
    setState(SystemState::IDLE);
}

void StateManager::handleImuRecoverySucceeded(const IMU_RecoverySucceeded& event) {
    ESP_LOGI(TAG, "IMU recovery succeeded!");
    if (m_currentState == SystemState::FATAL_ERROR) {
        ESP_LOGI(TAG, "Recovery succeeded while in FATAL_ERROR state. Returning to IDLE state.");
        setState(SystemState::IDLE);
    }
}

void StateManager::handleImuRecoveryFailed(const IMU_RecoveryFailed& event) {
    ESP_LOGE(TAG, "IMU recovery failed with error: %s (%d)",
             esp_err_to_name(event.errorCode), event.errorCode);
    ESP_LOGE(TAG, "IMU recovery exhausted all attempts. Setting system to FATAL_ERROR state.");
    setState(SystemState::FATAL_ERROR);
}

void StateManager::initiateCalibration(bool force) {
    ESP_LOGI(TAG, "Initiating calibration (force: %s)", force ? "true" : "false");

    if (m_currentState == SystemState::IDLE || force) {
        IMU_CalibrationRequest req_event;
        m_eventBus.publish(req_event);
        m_pending_calibration = false;
    } else if (m_currentState == SystemState::BALANCING && !force) {
        ESP_LOGW(TAG, "Cannot calibrate while BALANCING. Setting pending calibration flag.");
        m_pending_calibration = true;
        IMU_CalibrationRequestRejected reject_event(IMU_CalibrationRequestRejected::Reason::NOT_IDLE, true);
        m_eventBus.publish(reject_event);
    } else {
        ESP_LOGW(TAG, "Cannot calibrate from current state: %s", stateToString(m_currentState).c_str());
        IMU_CalibrationRequestRejected reject_event(IMU_CalibrationRequestRejected::Reason::OTHER, false);
        m_eventBus.publish(reject_event);
    }
}

void StateManager::handleCalibrationRejected(const IMU_CalibrationRequestRejected& event) {
    ESP_LOGW(TAG, "Calibration was rejected: %s",
             event.reason == IMU_CalibrationRequestRejected::Reason::NOT_IDLE ? "Not in IDLE state" :
             event.reason == IMU_CalibrationRequestRejected::Reason::RECOVERY_IN_PROGRESS ? "Recovery in progress" :
             "Other reason");

    if (event.retryWhenPossible) {
        ESP_LOGI(TAG, "Calibration will be retried when possible");
        m_pending_calibration = true;
    }
}
