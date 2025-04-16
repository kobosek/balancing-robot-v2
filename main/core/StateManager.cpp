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

// Other includes
#include "EventTypes.hpp"
#include "BaseEvent.hpp"
#include "EventBus.hpp"
#include "SystemState.hpp"
#include <cmath>
#include "esp_timer.h"
#include <string>
#include "esp_log.h"
#include "esp_err.h" // Include for esp_err_to_name

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static constexpr float RAD_TO_DEG = 180.0f / M_PI;

StateManager::StateManager(EventBus& eventBus) : // Removed IMUService dependency
    m_eventBus(eventBus),
    m_currentState(SystemState::INIT),
    m_withinRecoveryAngle(false),
    m_recoveryAngleStartTimeUs(0),
    m_autoRecoveryEnabled(true),
    m_fallDetectionEnabled(true),
    m_preImuRecoveryState(SystemState::IDLE),
    m_imu_recovery_attempts(0)
{}

esp_err_t StateManager::init() {
    ESP_LOGI(TAG, "Initializing StateManager...");
    // Init method might be empty now, or used for other setup if needed.
    // Subscriptions are handled in subscribeToEvents.
    ESP_LOGI(TAG, "StateManager Initialized.");
    return ESP_OK;
}

SystemState StateManager::getCurrentState() const {
    // Consider adding mutex if state is accessed from multiple tasks *without* going via event bus
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

        // Don't trigger recovery here, it's already triggered in handleImuCommunicationError
        // This prevents double-triggering of recovery
        // if (newState == SystemState::IMU_RECOVERY) {
        //    requestImuRecovery();
        // }
    }
}

// <<< ADDED: Event subscription logic >>>
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
    ESP_LOGI(TAG, "StateManager Subscriptions Complete.");
}

// --- Event Handlers ---

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
     // Allow starting from IDLE or FALLEN (if manual start desired after fall)
     // Prevent starting if calibrating or already balancing/init/error
     if (m_currentState == SystemState::IDLE || m_currentState == SystemState::FALLEN) {
          ESP_LOGI(TAG, "Transitioning to BALANCING state.");
          setState(SystemState::BALANCING);
     } else {
          ESP_LOGW(TAG, "Cannot start balancing from current state: %s", stateToString(m_currentState).c_str());
     }
}

void StateManager::handleStop(const StopCommand& event) {
    ESP_LOGI(TAG, "Stop Command Received by StateManager.");
    // Transition to IDLE from any state except INIT (unless specific logic needed for ERROR)
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
        // Consider publishing LOW_BATTERY_WARNING event as well
    }
}

void StateManager::handleOrientationUpdate(const OrientationDataEvent& event) {
    // Auto-recovery logic only applies when FALLEN
    if (m_currentState != SystemState::FALLEN) {
        // Reset tracker if we were potentially recovering but left FALLEN state manually
        if (m_withinRecoveryAngle) {
            ESP_LOGD(TAG, "Left FALLEN state, clearing recovery tracker.");
            m_withinRecoveryAngle = false;
            m_recoveryAngleStartTimeUs = 0;
        }
        return;
    }

    // --- Fallen State Recovery Logic ---
    if (!m_autoRecoveryEnabled) {
        // If recovery is disabled, ensure tracker is reset if it was active
         if (m_withinRecoveryAngle) {
             m_withinRecoveryAngle = false;
             m_recoveryAngleStartTimeUs = 0;
             ESP_LOGD(TAG,"Auto-recovery disabled, resetting potential recovery timer.");
         }
        return; // Don't proceed with recovery check
    }

    bool is_upright_now = (std::abs(event.pitch_rad) < RECOVERY_ANGLE_THRESHOLD_RAD);

    int64_t current_time_us = esp_timer_get_time();

    if (is_upright_now) {
        if (!m_withinRecoveryAngle) {
            // Just entered upright threshold
            m_withinRecoveryAngle = true;
            m_recoveryAngleStartTimeUs = current_time_us;
            ESP_LOGI(TAG, "Robot potentially upright in FALLEN state (P:%.1f), starting recovery timer (%llu us)...",
                     event.pitch_rad * RAD_TO_DEG, RECOVERY_HOLD_TIME_US);
        } else {
            // Still upright, check duration
            uint64_t time_held_upright_us = current_time_us - m_recoveryAngleStartTimeUs;
            if (time_held_upright_us >= RECOVERY_HOLD_TIME_US) {
                ESP_LOGI(TAG, "Robot held upright for %llu us. Auto-recovering to BALANCING state.", time_held_upright_us);
                // Reset trackers implicitly by leaving FALLEN state via setState
                setState(SystemState::BALANCING);
                // Note: If you prefer IDLE state after recovery, change the line above to:
                // setState(SystemState::IDLE);
            } else {
                 ESP_LOGV(TAG, "Holding upright... %llu / %llu us", time_held_upright_us, RECOVERY_HOLD_TIME_US);
            }
        }
    } else {
        // Not upright now
        if (m_withinRecoveryAngle) {
            // Just fell out of upright threshold
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
    // Only transition back to IDLE if we are *actually* in the CALIBRATING state.
    // This prevents interrupting an IMU_RECOVERY process that also involves calibration.
    if (m_currentState == SystemState::CALIBRATING_IMU) {
        ESP_LOGI(TAG, "Transitioning back to IDLE state after calibration.");
        setState(SystemState::IDLE);
        // TODO: Maybe notify user via web? (e.g., calibration status endpoint)
    } else {
         ESP_LOGW(TAG, "Received Calibration Complete event but not in CALIBRATING_IMU state (%s). Ignoring.", stateToString(m_currentState).c_str());
    }
}

void StateManager::handleImuCommunicationError(const IMU_CommunicationErrorEvent& event) {
    ESP_LOGE(TAG, "IMU Communication Error detected! Code: %s (%d). Entering IMU Recovery.",
             esp_err_to_name(event.errorCode), event.errorCode);

    // Store current state before entering recovery
    m_preImuRecoveryState = m_currentState;

    // Set state to IMU_RECOVERY
    setState(SystemState::IMU_RECOVERY);

    // Request IMU recovery via event bus
    requestImuRecovery();
}

// New handler for IMU recovery success
void StateManager::handleImuRecoverySucceeded(const ImuRecoverySucceededEvent& event) {
    // Determine appropriate state to return to
    SystemState returnState;

    // Never return to FATAL_ERROR state after successful recovery
    if (m_preImuRecoveryState == SystemState::FATAL_ERROR) {
        returnState = SystemState::IDLE;
    }
    // Special handling for BALANCING state - make sure we can safely return to it
    else if (m_preImuRecoveryState == SystemState::BALANCING) {
        // Check if we're within safe angles to resume balancing
        // This could be enhanced with actual angle checks if needed
        ESP_LOGI(TAG, "Previous state was BALANCING, will attempt to resume balancing");
        returnState = SystemState::BALANCING;
    }
    // For all other states, return to the previous state
    else {
        returnState = m_preImuRecoveryState;
    }

    ESP_LOGI(TAG, "IMU recovery succeeded! Previous state was: %s, returning to: %s",
             stateToString(m_preImuRecoveryState).c_str(),
             stateToString(returnState).c_str());

    // Reset recovery attempts counter
    m_imu_recovery_attempts = 0;

    // Return to appropriate state
    setState(returnState);
}

// New handler for IMU recovery failure
void StateManager::handleImuRecoveryFailed(const ImuRecoveryFailedEvent& event) {
    ESP_LOGE(TAG, "IMU recovery failed with error: %s (%d)",
             esp_err_to_name(event.errorCode), event.errorCode);

    // Handle recovery failure (retry or go to FATAL_ERROR)
    handleRecoveryFailure();
}

// Request IMU recovery via event bus instead of direct method call
void StateManager::requestImuRecovery() {
    ESP_LOGI(TAG, "Requesting IMU recovery (attempt %d of %d)",
             m_imu_recovery_attempts + 1, MAX_IMU_RECOVERY_ATTEMPTS);

    // Increment attempt counter
    m_imu_recovery_attempts++;

    // Publish recovery command event
    AttemptImuRecoveryCommand cmd;
    m_eventBus.publish(cmd);
}

void StateManager::handleRecoveryFailure() {
    // Check if we've reached the maximum number of attempts
    if (m_imu_recovery_attempts >= MAX_IMU_RECOVERY_ATTEMPTS) {
        ESP_LOGE(TAG, "Maximum IMU recovery attempts (%d) reached. Setting FATAL_ERROR state.",
                 MAX_IMU_RECOVERY_ATTEMPTS);
        setState(SystemState::FATAL_ERROR);
        return;
    }

    // Otherwise, wait and try again
    ESP_LOGW(TAG, "IMU recovery attempt %d failed. Will retry after delay...",
             m_imu_recovery_attempts);

    // In a real implementation, you might want to use a timer or task to delay
    // For simplicity, we'll request recovery again immediately
    requestImuRecovery();
}

void StateManager::handleEnableRecovery(const EnableRecoveryCommandEvent& event) {
    setAutoRecovery(true);
}

void StateManager::handleDisableRecovery(const DisableRecoveryCommandEvent& event) {
    setAutoRecovery(false);
    // Reset tracker if disabled while potentially recovering
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