// ================================================
// File: main/core/StateManager.cpp
// ================================================
#include "StateManager.hpp"

#include "BALANCE_FallDetected.hpp"
#include "BALANCE_AutoBalanceReady.hpp"
#include "BATTERY_StatusUpdate.hpp"
#include "BaseEvent.hpp"
#include "CONFIG_FullConfigUpdate.hpp"
#include "EventBus.hpp"
#include "IMU_AttachRequested.hpp"
#include "IMU_AvailabilityChanged.hpp"
#include "IMU_CalibrationCompleted.hpp"
#include "IMU_CalibrationRequest.hpp"
#include "IMU_CalibrationRequestRejected.hpp"
#include "IMU_CommunicationError.hpp"
#include "SYSTEM_StateChanged.hpp"
#include "UI_CalibrateImu.hpp"
#include "UI_CancelPidTuning.hpp"
#include "UI_StartPidTuning.hpp"
#include "UI_CancelGuidedCalibration.hpp"
#include "UI_StartGuidedCalibration.hpp"
#include "UI_StartBalancing.hpp"
#include "UI_Stop.hpp"
#include "PID_TuningFinished.hpp"
#include "GUIDED_CalibrationFinished.hpp"
#include "esp_err.h"
#include "esp_log.h"

StateManager::StateManager(EventBus& eventBus, const SystemBehaviorConfig& initialBehaviorConfig, const BatteryConfig& initialBatteryConfig) :
    m_eventBus(eventBus),
    m_currentState(SystemState::INIT) {
    applyConfig(initialBehaviorConfig, initialBatteryConfig);
}

esp_err_t StateManager::init() {
    ESP_LOGI(TAG, "Initializing StateManager...");
    ESP_LOGI(TAG, "StateManager Initialized.");
    return ESP_OK;
}

void StateManager::applyConfig(const SystemBehaviorConfig& behaviorConfig, const BatteryConfig& batteryConfig) {
    (void)behaviorConfig;
    m_criticalBatteryMotorShutdownEnabled = batteryConfig.critical_battery_motor_shutdown_enabled;
}

SystemState StateManager::getCurrentState() const {
    return m_currentState;
}

std::string StateManager::stateToString(SystemState state) const {
    switch (state) {
        case SystemState::INIT:
            return "INIT";
        case SystemState::IDLE:
            return "IDLE";
        case SystemState::BALANCING:
            return "BALANCING";
        case SystemState::PID_TUNING:
            return "PID_TUNING";
        case SystemState::GUIDED_CALIBRATION:
            return "GUIDED_CALIBRATION";
        case SystemState::FALLEN:
            return "FALLEN";
        case SystemState::SHUTDOWN:
            return "SHUTDOWN";
        case SystemState::FATAL_ERROR:
            return "FATAL_ERROR";
        default:
            return "UNKNOWN";
    }
}

void StateManager::setState(SystemState newState) {
    const SystemState previousState = m_currentState;
    bool stateChanged = false;
    if (m_currentState != newState) {
        m_currentState = newState;
        stateChanged = true;
        ESP_LOGI(TAG, "State changed from %d (%s) to %d (%s)",
                 static_cast<int>(previousState), stateToString(previousState).c_str(),
                 static_cast<int>(newState), stateToString(newState).c_str());

        if (previousState != SystemState::IDLE && newState == SystemState::IDLE && m_pending_calibration) {
            ESP_LOGD(TAG, "Entering IDLE state, checking for pending calibration.");
            initiateCalibration(true);
        }
    }

    if (stateChanged) {
        SYSTEM_StateChanged event(previousState, newState);
        m_eventBus.publish(event);
    }
}

void StateManager::handleEvent(const BaseEvent& event) {
    if (event.is<BALANCE_FallDetected>()) {
        handleFallDetected(event.as<BALANCE_FallDetected>());
    } else if (event.is<BALANCE_AutoBalanceReady>()) {
        handleAutoBalanceReady(event.as<BALANCE_AutoBalanceReady>());
    } else if (event.is<UI_StartBalancing>()) {
        handleStartBalancing(event.as<UI_StartBalancing>());
    } else if (event.is<UI_Stop>()) {
        handleStop(event.as<UI_Stop>());
    } else if (event.is<BATTERY_StatusUpdate>()) {
        handleBatteryUpdate(event.as<BATTERY_StatusUpdate>());
    } else if (event.is<UI_CalibrateImu>()) {
        handleCalibrateCommand(event.as<UI_CalibrateImu>());
    } else if (event.is<UI_StartPidTuning>()) {
        handleStartPidTuning(event.as<UI_StartPidTuning>());
    } else if (event.is<UI_CancelPidTuning>()) {
        handleCancelPidTuning(event.as<UI_CancelPidTuning>());
    } else if (event.is<PID_TuningFinished>()) {
        handlePidTuningFinished(event.as<PID_TuningFinished>());
    } else if (event.is<UI_StartGuidedCalibration>()) {
        handleStartGuidedCalibration(event.as<UI_StartGuidedCalibration>());
    } else if (event.is<UI_CancelGuidedCalibration>()) {
        handleCancelGuidedCalibration(event.as<UI_CancelGuidedCalibration>());
    } else if (event.is<GUIDED_CalibrationFinished>()) {
        handleGuidedCalibrationFinished(event.as<GUIDED_CalibrationFinished>());
    } else if (event.is<IMU_CalibrationCompleted>()) {
        handleCalibrationComplete(event.as<IMU_CalibrationCompleted>());
    } else if (event.is<IMU_CalibrationRequestRejected>()) {
        handleCalibrationRejected(event.as<IMU_CalibrationRequestRejected>());
    } else if (event.is<IMU_CommunicationError>()) {
        handleImuCommunicationError(event.as<IMU_CommunicationError>());
    } else if (event.is<IMU_AvailabilityChanged>()) {
        handleImuAvailabilityChanged(event.as<IMU_AvailabilityChanged>());
    } else if (event.is<CONFIG_FullConfigUpdate>()) {
        handleConfigUpdate(event.as<CONFIG_FullConfigUpdate>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

void StateManager::subscribeToEvents(EventBus& bus) {
    ESP_LOGW(TAG, "StateManager::subscribeToEvents is deprecated. Use EventBus::subscribe with EventHandler instead.");
    (void)bus;
}

void StateManager::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
    applyConfig(event.configData.behavior, event.configData.battery);
}

void StateManager::handleFallDetected(const BALANCE_FallDetected& event) {
    (void)event;
    ESP_LOGW(TAG, "Fall Detected Event Received!");
    if (m_currentState == SystemState::BALANCING) {
        setState(SystemState::FALLEN);
    }
}

void StateManager::handleAutoBalanceReady(const BALANCE_AutoBalanceReady& event) {
    (void)event;
    ESP_LOGI(TAG, "Auto balance ready event received.");

    if (m_currentState != SystemState::IDLE && m_currentState != SystemState::FALLEN) {
        return;
    }

    if (!m_imu_available) {
        ESP_LOGW(TAG, "Ignoring auto balance trigger because IMU is unavailable.");
        return;
    }

    if (m_criticalBatteryMotorShutdownEnabled && m_battery_critical) {
        ESP_LOGW(TAG, "Ignoring auto balance trigger because battery is critical and shutdown is enabled.");
        m_pending_start = false;
        if (m_currentState == SystemState::FALLEN) {
            setState(SystemState::IDLE);
        }
        return;
    }

    if (m_currentState == SystemState::FALLEN || m_currentState == SystemState::IDLE) {
        setState(SystemState::BALANCING);
    }
}

void StateManager::handleStartBalancing(const UI_StartBalancing& event) {
    (void)event;
    ESP_LOGI(TAG, "Start Balancing Command Received by StateManager.");

    if (m_currentState != SystemState::IDLE && m_currentState != SystemState::FALLEN) {
        ESP_LOGW(TAG, "Cannot start balancing from current state: %s", stateToString(m_currentState).c_str());
        return;
    }

    if (m_criticalBatteryMotorShutdownEnabled && m_battery_critical) {
        ESP_LOGW(TAG, "Cannot start balancing because battery is critical and motor shutdown is enabled.");
        m_pending_start = false;
        if (m_currentState == SystemState::FALLEN) {
            setState(SystemState::IDLE);
        }
        return;
    }

    if (!m_imu_available) {
        if (m_currentState == SystemState::FALLEN) {
            setState(SystemState::IDLE);
        }
        m_pending_start = true;
        m_eventBus.publish(IMU_AttachRequested());
        return;
    }

    setState(SystemState::BALANCING);
}

void StateManager::handleStop(const UI_Stop& event) {
    (void)event;
    ESP_LOGI(TAG, "Stop Command Received by StateManager.");
    m_pending_start = false;
    setState(SystemState::IDLE);
}

void StateManager::handleBatteryUpdate(const BATTERY_StatusUpdate& event) {
    m_battery_critical = event.status.isCritical;

    if (m_criticalBatteryMotorShutdownEnabled && event.status.isCritical) {
        m_pending_start = false;
        if (m_currentState == SystemState::BALANCING ||
            m_currentState == SystemState::PID_TUNING ||
            m_currentState == SystemState::GUIDED_CALIBRATION) {
            ESP_LOGW(TAG, "Battery level CRITICAL while motors active. Transitioning to IDLE.");
            setState(SystemState::IDLE);
        }
    }
}

void StateManager::handleCalibrateCommand(const UI_CalibrateImu& event) {
    (void)event;
    ESP_LOGI(TAG, "Calibrate Command Received.");
    initiateCalibration(false);
}

void StateManager::handleStartPidTuning(const UI_StartPidTuning& event) {
    (void)event;
    ESP_LOGI(TAG, "PID tuning command received.");

    if (m_currentState != SystemState::IDLE) {
        ESP_LOGW(TAG, "Cannot start PID tuning from current state: %s", stateToString(m_currentState).c_str());
        return;
    }

    if (m_criticalBatteryMotorShutdownEnabled && m_battery_critical) {
        ESP_LOGW(TAG, "Cannot start PID tuning because battery is critical and motor shutdown is enabled.");
        return;
    }

    setState(SystemState::PID_TUNING);
}

void StateManager::handleCancelPidTuning(const UI_CancelPidTuning& event) {
    (void)event;
    if (m_currentState == SystemState::PID_TUNING) {
        ESP_LOGI(TAG, "Canceling PID tuning.");
        setState(SystemState::IDLE);
    }
}

void StateManager::handlePidTuningFinished(const PID_TuningFinished& event) {
    ESP_LOGI(TAG, "PID tuning finished with state %d: %s",
             static_cast<int>(event.resultState),
             event.message.c_str());
    if (m_currentState == SystemState::PID_TUNING) {
        setState(SystemState::IDLE);
    }
}

void StateManager::handleStartGuidedCalibration(const UI_StartGuidedCalibration& event) {
    (void)event;
    ESP_LOGI(TAG, "Guided calibration command received.");

    if (m_currentState != SystemState::IDLE) {
        ESP_LOGW(TAG, "Cannot start guided calibration from current state: %s", stateToString(m_currentState).c_str());
        return;
    }

    if (!m_imu_available) {
        ESP_LOGW(TAG, "Cannot start guided calibration because IMU is unavailable.");
        m_eventBus.publish(IMU_AttachRequested());
        return;
    }

    if (m_criticalBatteryMotorShutdownEnabled && m_battery_critical) {
        ESP_LOGW(TAG, "Cannot start guided calibration because battery is critical and shutdown is enabled.");
        return;
    }

    setState(SystemState::GUIDED_CALIBRATION);
}

void StateManager::handleCancelGuidedCalibration(const UI_CancelGuidedCalibration& event) {
    (void)event;
    if (m_currentState == SystemState::GUIDED_CALIBRATION) {
        setState(SystemState::IDLE);
    }
}

void StateManager::handleGuidedCalibrationFinished(const GUIDED_CalibrationFinished& event) {
    ESP_LOGI(TAG, "Guided calibration finished: success=%s message=%s",
             event.success ? "true" : "false",
             event.message.c_str());
    if (m_currentState == SystemState::GUIDED_CALIBRATION) {
        setState(SystemState::IDLE);
    }
}

void StateManager::handleCalibrationComplete(const IMU_CalibrationCompleted& event) {
    ESP_LOGI(TAG, "Calibration Complete Event Received (Status: %s).", esp_err_to_name(event.status));
    setState(SystemState::IDLE);
}

void StateManager::handleImuCommunicationError(const IMU_CommunicationError& event) {
    ESP_LOGE(TAG, "IMU communication error: %s (%d)", esp_err_to_name(event.errorCode), event.errorCode);
    m_pending_start = false;

    if (m_currentState == SystemState::BALANCING || m_currentState == SystemState::FALLEN || m_currentState == SystemState::PID_TUNING) {
        setState(SystemState::IDLE);
    }
}

void StateManager::handleImuAvailabilityChanged(const IMU_AvailabilityChanged& event) {
    m_imu_available = event.available;
    ESP_LOGI(TAG, "IMU availability changed: %s", m_imu_available ? "available" : "unavailable");

    if (m_imu_available &&
        m_pending_start &&
        m_currentState == SystemState::IDLE &&
        (!m_criticalBatteryMotorShutdownEnabled || !m_battery_critical)) {
        m_pending_start = false;
        setState(SystemState::BALANCING);
    }
}

void StateManager::initiateCalibration(bool force) {
    ESP_LOGI(TAG, "Initiating calibration (force: %s)", force ? "true" : "false");

    if (m_currentState == SystemState::IDLE || force) {
        IMU_CalibrationRequest requestEvent;
        m_eventBus.publish(requestEvent);
        m_pending_calibration = false;
        return;
    }

    if ((m_currentState == SystemState::BALANCING ||
         m_currentState == SystemState::PID_TUNING ||
         m_currentState == SystemState::GUIDED_CALIBRATION) && !force) {
        m_pending_calibration = true;
        IMU_CalibrationRequestRejected rejectEvent(IMU_CalibrationRequestRejected::Reason::NOT_IDLE, true);
        m_eventBus.publish(rejectEvent);
        return;
    }

    IMU_CalibrationRequestRejected rejectEvent(IMU_CalibrationRequestRejected::Reason::OTHER, false);
    m_eventBus.publish(rejectEvent);
}

void StateManager::handleCalibrationRejected(const IMU_CalibrationRequestRejected& event) {
    ESP_LOGW(TAG, "Calibration was rejected.");
    if (event.retryWhenPossible) {
        m_pending_calibration = true;
    }
}
