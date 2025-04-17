// ================================================
// File: main/core/StateManager.cpp
// ================================================
#include "StateManager.hpp"

// Include all necessary event headers
#include "SystemStateChangedEvent.hpp"
#include "FallDetectionEvent.hpp"
#include "StartBalancingCommand.hpp"
#include "StopCommand.hpp"
#include "BatteryStatusUpdatedEvent.hpp"
#include "OrientationDataEvent.hpp"
#include "CalibrateCommandEvent.hpp"
#include "StartCalibrationRequestEvent.hpp"
#include "CalibrationCompleteEvent.hpp"
#include "EnableRecoveryCommandEvent.hpp"
#include "DisableRecoveryCommandEvent.hpp"
#include "EnableFallDetectCommandEvent.hpp"
#include "DisableFallDetectCommandEvent.hpp"
#include "IMU_CommunicationErrorEvent.hpp"
#include "ImuRecoveryEvents.hpp"
#include "ConfigUpdatedEvent.hpp" // Need to react to config changes

// Other includes
#include "EventTypes.hpp"
#include "BaseEvent.hpp"
#include "EventBus.hpp"
#include "SystemState.hpp"
#include "ConfigData.hpp" // Include config data definitions
#include <cmath>
#include "esp_timer.h"
#include <string>
#include "esp_log.h"
#include "esp_err.h" // Include for esp_err_to_name

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static constexpr float RAD_TO_DEG = 180.0f / M_PI;
static constexpr float DEG_TO_RAD = M_PI / 180.0f;


// Constructor takes initial config struct
StateManager::StateManager(EventBus& eventBus, const SystemBehaviorConfig& initialBehaviorConfig) :
    m_eventBus(eventBus),
    m_currentState(SystemState::INIT),
    m_withinRecoveryAngle(false),
    m_recoveryAngleStartTimeUs(0),
    m_autoRecoveryEnabled(true),
    m_fallDetectionEnabled(true),
    m_preImuRecoveryState(SystemState::IDLE),
    m_imu_recovery_attempts(0),
    // Initialize local config copies with defaults first
    m_recovery_angle_threshold_rad(0.087f), // ~5 deg
    m_recovery_hold_time_us(2000000),
    m_max_imu_recovery_attempts(3)
{
    applyConfig(initialBehaviorConfig); // Apply initial config
}

esp_err_t StateManager::init() {
    ESP_LOGI(TAG, "Initializing StateManager...");
    // No config loading here, applied in constructor
    ESP_LOGI(TAG, "StateManager Initialized.");
    return ESP_OK;
}

// Helper to apply values from a config struct
void StateManager::applyConfig(const SystemBehaviorConfig& config) {
    m_recovery_angle_threshold_rad = config.recovery_pitch_threshold_deg * DEG_TO_RAD;
    m_recovery_hold_time_us = config.recovery_hold_duration_ms * 1000ULL;
    m_max_imu_recovery_attempts = config.imu_recovery_max_attempts;
    // m_imu_recovery_delay_ms isn't directly used here, but could be if delays were added

    ESP_LOGI(TAG, "Applied StateManager params: RecovThresh=%.2f deg (%.3f rad), RecovHold=%llu us, MaxIMURecov=%d",
             config.recovery_pitch_threshold_deg, m_recovery_angle_threshold_rad, m_recovery_hold_time_us, m_max_imu_recovery_attempts);
}


SystemState StateManager::getCurrentState() const {
    return m_currentState;
}

bool StateManager::isAutoRecoveryEnabled() const {
    return m_autoRecoveryEnabled;
}

void StateManager::setAutoRecovery(bool enabled) {
    if (m_autoRecoveryEnabled != enabled) {
        m_autoRecoveryEnabled = enabled;
        ESP_LOGI(TAG, "Auto Recovery %s.", enabled ? "ENABLED" : "DISABLED");
    }
}

bool StateManager::isFallDetectionEnabled() const {
    return m_fallDetectionEnabled;
}

void StateManager::enableFallDetection(bool enabled) {
     if (m_fallDetectionEnabled != enabled) {
        m_fallDetectionEnabled = enabled;
        ESP_LOGI(TAG, "Fall Detection Transition %s.", enabled ? "ENABLED" : "DISABLED");
    }
}

std::string StateManager::stateToString(SystemState state) const {
     switch(state) {
          case SystemState::INIT: return "INIT";
          case SystemState::IDLE: return "IDLE";
          case SystemState::CALIBRATING_IMU: return "CALIBRATING_IMU";
          case SystemState::BALANCING: return "BALANCING";
          case SystemState::FALLEN: return "FALLEN";
          case SystemState::IMU_RECOVERY: return "IMU_RECOVERY";
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

        // Reset recovery tracking when leaving FALLEN state
        if (previousState == SystemState::FALLEN && newState != SystemState::FALLEN) {
             ESP_LOGD(TAG, "Leaving FALLEN state, resetting recovery tracker.");
             m_withinRecoveryAngle = false;
             m_recoveryAngleStartTimeUs = 0;
        }

        // Reset IMU recovery attempt counter when leaving recovery state
        if (previousState == SystemState::IMU_RECOVERY && newState != SystemState::IMU_RECOVERY) {
            ESP_LOGD(TAG, "Leaving IMU_RECOVERY state, resetting attempt counter.");
            m_imu_recovery_attempts = 0;
        }
    }

    if (stateChanged) {
        SystemStateChangedEvent event(previousState, newState);
        m_eventBus.publish(event);
    }
}

void StateManager::subscribeToEvents(EventBus& bus) {
    ESP_LOGI(TAG, "Subscribing StateManager to events...");

    // Essential state transitions & commands
    bus.subscribe(EventType::FALL_DETECTED, [this](const BaseEvent& ev){ this->handleFallDetected(static_cast<const FallDetectionEvent&>(ev)); });
    bus.subscribe(EventType::START_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleStartBalancing(static_cast<const StartBalancingCommand&>(ev)); });
    bus.subscribe(EventType::STOP_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleStop(static_cast<const StopCommand&>(ev)); });
    bus.subscribe(EventType::BATTERY_STATUS_UPDATED, [this](const BaseEvent& ev){ this->handleBatteryUpdate(static_cast<const BatteryStatusUpdatedEvent&>(ev)); });

    // Auto-recovery related
    bus.subscribe(EventType::ORIENTATION_DATA_READY, [this](const BaseEvent& ev){ this->handleOrientationUpdate(static_cast<const OrientationDataEvent&>(ev)); });
    bus.subscribe(EventType::ENABLE_RECOVERY_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleEnableRecovery(static_cast<const EnableRecoveryCommandEvent&>(ev)); });
    bus.subscribe(EventType::DISABLE_RECOVERY_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleDisableRecovery(static_cast<const DisableRecoveryCommandEvent&>(ev)); });

    // Fall detection enable/disable
    bus.subscribe(EventType::ENABLE_FALL_DETECT_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleEnableFallDetect(static_cast<const EnableFallDetectCommandEvent&>(ev)); });
    bus.subscribe(EventType::DISABLE_FALL_DETECT_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleDisableFallDetect(static_cast<const DisableFallDetectCommandEvent&>(ev)); });

    // Calibration related
    bus.subscribe(EventType::CALIBRATE_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleCalibrateCommand(static_cast<const CalibrateCommandEvent&>(ev)); });
    bus.subscribe(EventType::CALIBRATION_COMPLETE, [this](const BaseEvent& ev){ this->handleCalibrationComplete(static_cast<const CalibrationCompleteEvent&>(ev)); });

    // IMU health/recovery related
    bus.subscribe(EventType::IMU_COMMUNICATION_ERROR, [this](const BaseEvent& ev){ this->handleImuCommunicationError(static_cast<const IMU_CommunicationErrorEvent&>(ev)); });
    bus.subscribe(EventType::IMU_RECOVERY_SUCCEEDED, [this](const BaseEvent& ev){ this->handleImuRecoverySucceeded(static_cast<const ImuRecoverySucceededEvent&>(ev)); });
    bus.subscribe(EventType::IMU_RECOVERY_FAILED, [this](const BaseEvent& ev){ this->handleImuRecoveryFailed(static_cast<const ImuRecoveryFailedEvent&>(ev)); });

    // Config Updates
    bus.subscribe(EventType::CONFIG_UPDATED, [this](const BaseEvent& ev){
        ESP_LOGD(TAG, "Config update received, applying new parameters.");
        this->handleConfigUpdate(ev); // Call the handler
    });

    ESP_LOGI(TAG, "StateManager Subscriptions Complete.");
}

// --- Event Handlers ---

// --- ADDED Config Update Handler ---
void StateManager::handleConfigUpdate(const BaseEvent& event) {
    if (event.type != EventType::CONFIG_UPDATED) return;
    const auto& configEvent = static_cast<const ConfigUpdatedEvent&>(event);
    ESP_LOGI(TAG, "Handling config update event.");
    // Extract the relevant part and apply it
    applyConfig(configEvent.configData.behavior);
}
// --- END ADDED ---

void StateManager::handleFallDetected(const FallDetectionEvent& event) {
    ESP_LOGW(TAG, "Fall Detected Event Received!");
    if (!m_fallDetectionEnabled) {
         ESP_LOGI(TAG, "Fall detection transition is disabled, ignoring fall event.");
         return;
    }
    if (m_currentState == SystemState::BALANCING) {
        ESP_LOGW(TAG, "Transitioning to FALLEN state due to fall detection.");
        setState(SystemState::FALLEN);
    } else {
        ESP_LOGI(TAG, "Fall detected but not in BALANCING state (%s), ignoring.", stateToString(m_currentState).c_str());
    }
}

void StateManager::handleStartBalancing(const StartBalancingCommand& event) {
     ESP_LOGI(TAG, "Start Balancing Command Received by StateManager.");
     if (m_currentState == SystemState::IDLE || m_currentState == SystemState::FALLEN) {
          ESP_LOGI(TAG, "Transitioning to BALANCING state.");
          setState(SystemState::BALANCING);
     } else {
          ESP_LOGW(TAG, "Cannot start balancing from current state: %s", stateToString(m_currentState).c_str());
     }
}

void StateManager::handleStop(const StopCommand& event) {
    ESP_LOGI(TAG, "Stop Command Received by StateManager.");
    if (m_currentState != SystemState::IDLE && m_currentState != SystemState::INIT) {
         ESP_LOGI(TAG, "Transitioning to IDLE state from %s.", stateToString(m_currentState).c_str());
         setState(SystemState::IDLE);
    } else {
        ESP_LOGI(TAG, "Stop command ignored, already in %s state.", stateToString(m_currentState).c_str());
        setState(SystemState::IDLE); // Ensure it's IDLE
    }
}

void StateManager::handleBatteryUpdate(const BatteryStatusUpdatedEvent& event) {
    ESP_LOGV(TAG, "Battery Update Received: %.2fV, %d%%", event.status.voltage, event.status.percentage);
    if (event.status.isLow && m_currentState == SystemState::BALANCING) {
        ESP_LOGW(TAG, "Battery level LOW while balancing! Transitioning to IDLE.");
        setState(SystemState::IDLE);
    }
}

void StateManager::handleOrientationUpdate(const OrientationDataEvent& event) {
    if (m_currentState != SystemState::FALLEN) {
        if (m_withinRecoveryAngle) {
            ESP_LOGD(TAG, "Left FALLEN state, clearing recovery tracker.");
            m_withinRecoveryAngle = false;
            m_recoveryAngleStartTimeUs = 0;
        }
        return;
    }
    if (!m_autoRecoveryEnabled) {
         if (m_withinRecoveryAngle) {
             m_withinRecoveryAngle = false;
             m_recoveryAngleStartTimeUs = 0;
             ESP_LOGD(TAG,"Auto-recovery disabled, resetting potential recovery timer.");
         }
        return;
    }

    // Use configured threshold
    bool is_upright_now = (std::abs(event.pitch_rad) < m_recovery_angle_threshold_rad);
    int64_t current_time_us = esp_timer_get_time();

    if (is_upright_now) {
        if (!m_withinRecoveryAngle) {
            m_withinRecoveryAngle = true;
            m_recoveryAngleStartTimeUs = current_time_us;
            ESP_LOGI(TAG, "Robot potentially upright in FALLEN state (P:%.1f), starting recovery timer (%llu us)...",
                     event.pitch_rad * RAD_TO_DEG, m_recovery_hold_time_us);
        } else {
            uint64_t time_held_upright_us = current_time_us - m_recoveryAngleStartTimeUs;
            // Use configured hold time
            if (time_held_upright_us >= m_recovery_hold_time_us) {
                ESP_LOGI(TAG, "Robot held upright for %llu us. Auto-recovering to BALANCING state.", time_held_upright_us);
                setState(SystemState::BALANCING);
            } else {
                 ESP_LOGV(TAG, "Holding upright... %llu / %llu us", time_held_upright_us, m_recovery_hold_time_us);
            }
        }
    } else {
        if (m_withinRecoveryAngle) {
            ESP_LOGI(TAG, "Robot moved out of recovery angle threshold (P:%.1f). Resetting recovery timer.",
                     event.pitch_rad * RAD_TO_DEG);
            m_withinRecoveryAngle = false;
            m_recoveryAngleStartTimeUs = 0;
        }
    }
}

void StateManager::handleCalibrateCommand(const CalibrateCommandEvent& event) {
    ESP_LOGI(TAG, "Calibrate Command Received.");
    if (m_currentState == SystemState::IDLE) {
        ESP_LOGI(TAG, "Transitioning to CALIBRATING_IMU state and requesting calibration.");
        setState(SystemState::CALIBRATING_IMU);
        StartCalibrationRequestEvent req_event;
        m_eventBus.publish(req_event);
    } else {
        ESP_LOGW(TAG, "Cannot start calibration from current state: %s", stateToString(m_currentState).c_str());
    }
}

void StateManager::handleCalibrationComplete(const CalibrationCompleteEvent& event) {
    ESP_LOGI(TAG, "Calibration Complete Event Received (Status: %s).", esp_err_to_name(event.status));
    if (m_currentState == SystemState::CALIBRATING_IMU) {
        ESP_LOGI(TAG, "Transitioning back to IDLE state after calibration.");
        setState(SystemState::IDLE);
    } else {
         ESP_LOGW(TAG, "Received Calibration Complete event but not in CALIBRATING_IMU state (%s). Ignoring.", stateToString(m_currentState).c_str());
    }
}

void StateManager::handleImuCommunicationError(const IMU_CommunicationErrorEvent& event) {
    ESP_LOGE(TAG, "IMU Communication Error detected! Code: %s (%d). Entering IMU Recovery.",
             esp_err_to_name(event.errorCode), event.errorCode);
    m_preImuRecoveryState = m_currentState;
    setState(SystemState::IMU_RECOVERY);
    requestImuRecovery();
}

void StateManager::handleImuRecoverySucceeded(const ImuRecoverySucceededEvent& event) {
    SystemState returnState;
    if (m_preImuRecoveryState == SystemState::FATAL_ERROR) {
        returnState = SystemState::IDLE;
    } else if (m_preImuRecoveryState == SystemState::BALANCING) {
        ESP_LOGI(TAG, "Previous state was BALANCING, will attempt to resume balancing");
        returnState = SystemState::BALANCING;
    } else {
        returnState = m_preImuRecoveryState;
    }
    ESP_LOGI(TAG, "IMU recovery succeeded! Previous state was: %s, returning to: %s",
             stateToString(m_preImuRecoveryState).c_str(),
             stateToString(returnState).c_str());
    m_imu_recovery_attempts = 0;
    setState(returnState);
}

void StateManager::handleImuRecoveryFailed(const ImuRecoveryFailedEvent& event) {
    ESP_LOGE(TAG, "IMU recovery failed with error: %s (%d)",
             esp_err_to_name(event.errorCode), event.errorCode);
    handleRecoveryFailure();
}

void StateManager::requestImuRecovery() {
    // Use configured max attempts
    ESP_LOGI(TAG, "Requesting IMU recovery (attempt %d of %d)",
             m_imu_recovery_attempts + 1, m_max_imu_recovery_attempts);
    m_imu_recovery_attempts++;
    AttemptImuRecoveryCommand cmd;
    m_eventBus.publish(cmd);
}

void StateManager::handleRecoveryFailure() {
    // Use configured max attempts
    if (m_imu_recovery_attempts >= m_max_imu_recovery_attempts) {
        ESP_LOGE(TAG, "Maximum IMU recovery attempts (%d) reached. Setting FATAL_ERROR state.",
                 m_max_imu_recovery_attempts);
        setState(SystemState::FATAL_ERROR);
        return;
    }
    ESP_LOGW(TAG, "IMU recovery attempt %d failed. Will retry after delay...",
             m_imu_recovery_attempts);
    // The delay is handled implicitly by the health monitor triggering the next error
    requestImuRecovery(); // Triggering next attempt immediately after failure might be too aggressive
                          // Consider adding a delay here or relying on the health monitor
}

void StateManager::handleEnableRecovery(const EnableRecoveryCommandEvent& event) {
    setAutoRecovery(true);
}

void StateManager::handleDisableRecovery(const DisableRecoveryCommandEvent& event) {
    setAutoRecovery(false);
    if (m_withinRecoveryAngle) {
        m_withinRecoveryAngle = false;
        m_recoveryAngleStartTimeUs = 0;
        ESP_LOGD(TAG,"Auto-recovery disabled, resetting active recovery timer.");
    }
}

void StateManager::handleEnableFallDetect(const EnableFallDetectCommandEvent& event) {
    enableFallDetection(true);
}

void StateManager::handleDisableFallDetect(const DisableFallDetectCommandEvent& event) {
    enableFallDetection(false);
}