// main/state/StateManager.cpp
#include "StateManager.hpp"

// Include all necessary event headers
#include "SystemStateChangedEvent.hpp"
#include "FallDetectionEvent.hpp"
#include "StartBalancingCommand.hpp"
#include "StopCommand.hpp"
#include "BatteryStatusUpdatedEvent.hpp"
#include "OrientationDataEvent.hpp"
#include "CalibrateCommandEvent.hpp"
#include "StartCalibrationRequestEvent.hpp" // To publish
#include "CalibrationCompleteEvent.hpp"
#include "EnableRecoveryCommandEvent.hpp"
#include "DisableRecoveryCommandEvent.hpp"
#include "EnableFallDetectCommandEvent.hpp" // <<< ADDED
#include "DisableFallDetectCommandEvent.hpp" // <<< ADDED
#include "IMU_CommunicationErrorEvent.hpp" // <<< ADDED

// Other includes
#include "EventTypes.hpp"
#include "BaseEvent.hpp"
#include "EventBus.hpp"
#include "SystemState.hpp"
#include "IMUService.hpp" // <<< ADDED explicit include
#include <cmath>
#include "esp_timer.h"
#include <string>
#include "esp_log.h"
#include "esp_err.h" // Include for esp_err_to_name

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static constexpr float RAD_TO_DEG = 180.0f / M_PI;

// Forward declare IMUService if not included via header (it should be via StateManager.hpp now)
// class IMUService; // No longer needed if included above

StateManager::StateManager(EventBus& eventBus, IMUService& imuService) : // <<< MODIFIED
    m_eventBus(eventBus),
    m_imuService(imuService), // <<< ADDED
    m_currentState(SystemState::INIT),
    m_withinRecoveryAngle(false),
    m_recoveryAngleStartTimeUs(0),
    m_autoRecoveryEnabled(true), // Initialize new member
    m_fallDetectionEnabled(true), // Initialize new member
    m_preImuRecoveryState(SystemState::IDLE), // Initialize new member
    m_imu_recovery_attempts(0) // Initialize new member
{}

esp_err_t StateManager::init() {
    ESP_LOGI(TAG, "Initializing StateManager and subscribing to events...");
    // Existing subscriptions...
    m_eventBus.subscribe(EventType::FALL_DETECTED, [this](const BaseEvent& ev){ this->handleFallDetected(static_cast<const FallDetectionEvent&>(ev)); });
    m_eventBus.subscribe(EventType::START_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleStartBalancing(static_cast<const StartBalancingCommand&>(ev)); });
    m_eventBus.subscribe(EventType::STOP_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleStop(static_cast<const StopCommand&>(ev)); });
    m_eventBus.subscribe(EventType::BATTERY_STATUS_UPDATED, [this](const BaseEvent& ev){ this->handleBatteryUpdate(static_cast<const BatteryStatusUpdatedEvent&>(ev)); });
    m_eventBus.subscribe(EventType::ORIENTATION_DATA_READY, [this](const BaseEvent& ev){ this->handleOrientationUpdate(static_cast<const OrientationDataEvent&>(ev)); }); // Keep this!
    m_eventBus.subscribe(EventType::CALIBRATE_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleCalibrateCommand(static_cast<const CalibrateCommandEvent&>(ev)); });
    m_eventBus.subscribe(EventType::CALIBRATION_COMPLETE, [this](const BaseEvent& ev){ this->handleCalibrationComplete(static_cast<const CalibrationCompleteEvent&>(ev)); });
    m_eventBus.subscribe(EventType::ENABLE_RECOVERY_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleEnableRecovery(static_cast<const EnableRecoveryCommandEvent&>(ev)); });
    m_eventBus.subscribe(EventType::DISABLE_RECOVERY_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleDisableRecovery(static_cast<const DisableRecoveryCommandEvent&>(ev)); });

    // Subscriptions for fall detection enable/disable <<< ADDED
    m_eventBus.subscribe(EventType::ENABLE_FALL_DETECT_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleEnableFallDetect(static_cast<const EnableFallDetectCommandEvent&>(ev)); });
    m_eventBus.subscribe(EventType::DISABLE_FALL_DETECT_COMMAND_RECEIVED, [this](const BaseEvent& ev){ this->handleDisableFallDetect(static_cast<const DisableFallDetectCommandEvent&>(ev)); });

    // Subscribe to IMU communication errors <<< ADDED
    m_eventBus.subscribe(EventType::IMU_COMMUNICATION_ERROR, [this](const BaseEvent& ev){ this->handleImuCommunicationError(static_cast<const IMU_CommunicationErrorEvent&>(ev)); });

    ESP_LOGI(TAG, "StateManager Initialized and subscribed.");
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

// <<< ADDED >>>
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
          case SystemState::CALIBRATING_IMU: return "CALIBRATING_IMU"; // <<< ADDED
          case SystemState::BALANCING: return "BALANCING";
          case SystemState::FALLEN: return "FALLEN";
          case SystemState::IMU_RECOVERY: return "IMU_RECOVERY"; // <<< ADDED
          case SystemState::FATAL_ERROR: return "FATAL_ERROR"; // <<< MODIFIED
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

        // Trigger recovery sequence if entering IMU_RECOVERY state
        if (newState == SystemState::IMU_RECOVERY) {
            // NOTE: This recovery logic runs synchronously within the setState call.
            // If recovery takes too long, consider moving it to a separate task
            // signaled by the state change event. For simplicity, we do it here.
            attemptImuRecovery();
        }
    }
}

// --- Event Handlers ---

void StateManager::handleFallDetected(const FallDetectionEvent& event) {
    ESP_LOGW(TAG, "Fall Detected Event Received!");
    if (!m_fallDetectionEnabled) { // <<< ADDED Check
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
    ESP_LOGE(TAG, "IMU Communication Error Event Received!");
    // Avoid re-entering recovery if already recovering or in fatal error
    if (m_currentState == SystemState::IMU_RECOVERY || m_currentState == SystemState::FATAL_ERROR) {
        ESP_LOGW(TAG, "Already in recovery or fatal error state (%s), ignoring IMU error event.", stateToString(m_currentState).c_str());
        return;
    }

    ESP_LOGW(TAG, "Transitioning to IMU_RECOVERY state from %s.", stateToString(m_currentState).c_str());
    m_preImuRecoveryState = m_currentState; // Store state before recovery
    m_imu_recovery_attempts = 0; // Reset attempt counter
    setState(SystemState::IMU_RECOVERY);
    // The actual recovery attempt logic is triggered within setState now
}

// --- Private Helper Methods ---

void StateManager::attemptImuRecovery() {
    ESP_LOGI(TAG, "--- Starting IMU Recovery Attempt %d/%d ---", m_imu_recovery_attempts + 1, MAX_IMU_RECOVERY_ATTEMPTS);

    // Ensure motors are off (should be handled by RobotController based on state, but belt-and-suspenders)
    // m_motorService.setMotorEffort(0, 0); // Requires MotorService reference if done here

    m_imu_recovery_attempts++;
    esp_err_t ret = ESP_OK;

    // 1. Reset Sensor
    ret = m_imuService.resetSensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Recovery Step 1/4: Sensor Reset Failed (%s).", esp_err_to_name(ret));
        handleRecoveryFailure(); // Call failure handler
        return; // Exit after handling failure
    }
    ESP_LOGI(TAG, "Recovery Step 1/4: Sensor Reset OK.");

    // 2. Reinitialize Sensor
    ret = m_imuService.reinitializeSensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Recovery Step 2/4: Sensor Reinitialize Failed (%s).", esp_err_to_name(ret));
        handleRecoveryFailure(); // Call failure handler
        return; // Exit after handling failure
    }
    ESP_LOGI(TAG, "Recovery Step 2/4: Sensor Reinitialize OK.");

    // 3. Perform Calibration (Optional but recommended after reset/reinit)
    // Note: This blocks for the duration of calibration.
    ret = m_imuService.triggerCalibration(); // Use triggerCalibration to handle mutex & publish event
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Recovery Step 3/4: Sensor Calibration Failed (%s).", esp_err_to_name(ret));
        // Decide if calibration failure is fatal for recovery or just a warning
        // For now, let's treat it as fatal for this attempt.
        handleRecoveryFailure(); // Call failure handler
        return; // Exit after handling failure
    }
     ESP_LOGI(TAG, "Recovery Step 3/4: Sensor Calibration OK.");

    // 4. Verify Communication
    ret = m_imuService.verifyCommunication();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Recovery Step 4/4: Communication Verification Failed (%s).", esp_err_to_name(ret));
        handleRecoveryFailure(); // Call failure handler
        return; // Exit after handling failure
    }
    ESP_LOGI(TAG, "Recovery Step 4/4: Communication Verification OK.");

    // --- Recovery Succeeded --- (If we reach here, all steps passed)
    ESP_LOGI(TAG, "--- IMU Recovery Attempt %d Successful! ---", m_imu_recovery_attempts);
    ESP_LOGI(TAG, "Transitioning back to previous state: %s", stateToString(m_preImuRecoveryState).c_str());
    // Reset counter implicitly by leaving state via setState
    setState(m_preImuRecoveryState); // Transition back
}


// --- NEW: Helper to handle recovery failure logic ---
void StateManager::handleRecoveryFailure() {
    ESP_LOGW(TAG, "--- IMU Recovery Attempt %d Failed. ---", m_imu_recovery_attempts);
    if (m_imu_recovery_attempts >= MAX_IMU_RECOVERY_ATTEMPTS) {
        ESP_LOGE(TAG, "Maximum IMU recovery attempts reached (%d). Transitioning to FATAL_ERROR.", MAX_IMU_RECOVERY_ATTEMPTS);
        setState(SystemState::FATAL_ERROR);
    } else {
        ESP_LOGI(TAG, "Waiting %lu ms before next recovery attempt...", IMU_RECOVERY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(IMU_RECOVERY_DELAY_MS));
        // Re-trigger the attempt (recursive call)
        // Note: This recursive call is simple but increases stack depth.
        // For a small number of retries (like 3), this is usually fine.
        // If MAX_IMU_RECOVERY_ATTEMPTS were large, a loop or separate task might be better.
        attemptImuRecovery();
    }
}
// --- END NEW HELPER ---


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