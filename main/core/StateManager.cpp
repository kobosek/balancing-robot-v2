// ================================================
// File: main/core/StateManager.cpp
// ================================================
#include "StateManager.hpp"

// Include all necessary event headers
#include "SYSTEM_StateChanged.hpp"
#include "MOTION_FallDetected.hpp"
#include "UI_StartBalancing.hpp"
#include "UI_Stop.hpp"
#include "BATTERY_StatusUpdate.hpp"
#include "IMU_OrientationData.hpp"
#include "UI_CalibrateImu.hpp"
#include "IMU_CalibrationRequest.hpp"
#include "IMU_CalibrationCompleted.hpp"
#include "UI_EnableFallRecovery.hpp"
#include "UI_DisableFallRecovery.hpp"
#include "UI_EnableFallDetection.hpp"
#include "UI_DisableFallDetection.hpp"
#include "IMU_CommunicationError.hpp"
#include "CONFIG_FullConfigUpdate.hpp" // Need to react to config changes
#include "IMU_CalibrationRequestRejected.hpp"
#include "IMU_AttemptRecovery.hpp"
#include "IMU_RecoveryFailed.hpp"
#include "IMU_RecoverySucceeded.hpp"

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

        // Reset recovery tracking when leaving FALLEN state
        if (previousState == SystemState::FALLEN && newState != SystemState::FALLEN) {
             ESP_LOGD(TAG, "Leaving FALLEN state, resetting recovery tracker.");
             m_withinRecoveryAngle = false;
             m_recoveryAngleStartTimeUs = 0;
        }

        // Check for transitions to/from IDLE state
        if (previousState != SystemState::IDLE && newState == SystemState::IDLE) {
            ESP_LOGD(TAG, "Entering IDLE state, checking for pending calibration.");
            // If we have a pending calibration, initiate it now
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

void StateManager::subscribeToEvents(EventBus& bus) {
    ESP_LOGI(TAG, "Subscribing StateManager to events...");

    // Essential state transitions & commands
    bus.subscribe(EventType::MOTION_FALL_DETECTED, [this](const BaseEvent& ev){ this->handleFallDetected(static_cast<const MOTION_FallDetected&>(ev)); });
    bus.subscribe(EventType::UI_START_BALANCING, [this](const BaseEvent& ev){ this->handleStartBalancing(static_cast<const UI_StartBalancing&>(ev)); });
    bus.subscribe(EventType::UI_STOP, [this](const BaseEvent& ev){ this->handleStop(static_cast<const UI_Stop&>(ev)); });
    bus.subscribe(EventType::BATTERY_STATUS_UPDATE, [this](const BaseEvent& ev){ this->handleBatteryUpdate(static_cast<const BATTERY_StatusUpdate&>(ev)); });

    // Auto-recovery related
    bus.subscribe(EventType::IMU_ORIENTATION_DATA, [this](const BaseEvent& ev){ this->handleOrientationUpdate(static_cast<const IMU_OrientationData&>(ev)); });
    bus.subscribe(EventType::UI_ENABLE_FALL_RECOVERY, [this](const BaseEvent& ev){ this->handleEnableRecovery(static_cast<const UI_EnableFallRecovery&>(ev)); });
    bus.subscribe(EventType::UI_DISABLE_FALL_RECOVERY, [this](const BaseEvent& ev){ this->handleDisableRecovery(static_cast<const UI_DisableFallRecovery&>(ev)); });

    // Fall detection enable/disable
    bus.subscribe(EventType::UI_ENABLE_FALL_DETECTION, [this](const BaseEvent& ev){ this->handleEnableFallDetect(static_cast<const UI_EnableFallDetection&>(ev)); });
    bus.subscribe(EventType::UI_DISABLE_FALL_DETECTION, [this](const BaseEvent& ev){ this->handleDisableFallDetect(static_cast<const UI_DisableFallDetection&>(ev)); });

    // Calibration related
    bus.subscribe(EventType::UI_CALIBRATE_IMU, [this](const BaseEvent& ev){ this->handleCalibrateCommand(static_cast<const UI_CalibrateImu&>(ev)); });
    bus.subscribe(EventType::IMU_CALIBRATION_COMPLETED, [this](const BaseEvent& ev){ this->handleCalibrationComplete(static_cast<const IMU_CalibrationCompleted&>(ev)); });
    bus.subscribe(EventType::IMU_CALIBRATION_REJECTED, [this](const BaseEvent& ev){ this->handleCalibrationRejected(static_cast<const IMU_CalibrationRequestRejected&>(ev)); });

    // IMU health/recovery related
    bus.subscribe(EventType::IMU_COMMUNICATION_ERROR, [this](const BaseEvent& ev){ this->handleImuCommunicationError(static_cast<const IMU_CommunicationError&>(ev)); });
    bus.subscribe(EventType::IMU_RECOVERY_SUCCEEDED, [this](const BaseEvent& ev){ this->handleImuRecoverySucceeded(static_cast<const IMU_RecoverySucceeded&>(ev)); });
    bus.subscribe(EventType::IMU_RECOVERY_FAILED, [this](const BaseEvent& ev){ this->handleImuRecoveryFailed(static_cast<const IMU_RecoveryFailed&>(ev)); });

    // Config Updates
    bus.subscribe(EventType::CONFIG_FULL_UPDATE, [this](const BaseEvent& ev){
        ESP_LOGD(TAG, "Config update received, applying new parameters.");
        this->handleConfigUpdate(ev); // Call the handler
    });

    ESP_LOGI(TAG, "StateManager Subscriptions Complete.");
}

// --- Event Handlers ---

// --- ADDED Config Update Handler ---
void StateManager::handleConfigUpdate(const BaseEvent& event) {
    if (event.type != EventType::CONFIG_FULL_UPDATE) return;
    const auto& configEvent = static_cast<const CONFIG_FullConfigUpdate&>(event);
    ESP_LOGI(TAG, "Handling config update event.");
    // Extract the relevant part and apply it
    applyConfig(configEvent.configData.behavior);
}
// --- END ADDED ---

void StateManager::handleFallDetected(const MOTION_FallDetected& event) {
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
         setState(SystemState::IDLE);
    } else {
        ESP_LOGI(TAG, "Stop command ignored, already in %s state.", stateToString(m_currentState).c_str());
        setState(SystemState::IDLE); // Ensure it's IDLE
    }
}

void StateManager::handleBatteryUpdate(const BATTERY_StatusUpdate& event) {
    ESP_LOGV(TAG, "Battery Update Received: %.2fV, %d%%", event.status.voltage, event.status.percentage);
    if (event.status.isLow && m_currentState == SystemState::BALANCING) {
        ESP_LOGW(TAG, "Battery level LOW while balancing! Transitioning to IDLE.");
        setState(SystemState::IDLE);
    }
}

void StateManager::handleOrientationUpdate(const IMU_OrientationData& event) {
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

void StateManager::handleCalibrateCommand(const UI_CalibrateImu& event) {
    ESP_LOGI(TAG, "Calibrate Command Received.");
    initiateCalibration(false); // Use the new calibration method
}

void StateManager::handleCalibrationComplete(const IMU_CalibrationCompleted& event) {
    ESP_LOGI(TAG, "Calibration Complete Event Received (Status: %s).", esp_err_to_name(event.status));
    // No need to check for CALIBRATING_IMU state as it's now handled by the IMUService
    // Just ensure we're in IDLE state after calibration
    setState(SystemState::IDLE);
}

void StateManager::handleImuCommunicationError(const IMU_CommunicationError& event) {
    ESP_LOGE(TAG, "IMU Communication Error detected! Code: %s (%d).",
             esp_err_to_name(event.errorCode), event.errorCode);
    
    // Store current state before recovery
    m_preImuRecoveryState = m_currentState;
    
    // Initiate IMU recovery with minimal interruption if we're balancing
    bool minimal_interruption = (m_currentState == SystemState::BALANCING);
    initiateIMURecovery(minimal_interruption);
}

void StateManager::handleImuRecoverySucceeded(const IMU_RecoverySucceeded& event) {
    // Just reset recovery attempts counter - no need to change system state
    // since it wasn't changed for recovery
    ESP_LOGI(TAG, "IMU recovery succeeded!");
    m_imu_recovery_attempts = 0;
}

void StateManager::handleImuRecoveryFailed(const IMU_RecoveryFailed& event) {
    ESP_LOGE(TAG, "IMU recovery failed with error: %s (%d)",
             esp_err_to_name(event.errorCode), event.errorCode);
    handleRecoveryFailure();
}

void StateManager::initiateIMURecovery(bool minimal_interruption) {
    ESP_LOGI(TAG, "Initiating IMU recovery (minimal_interruption: %s)", minimal_interruption ? "true" : "false");
    
    // Reset recovery attempt counter for a new recovery session
    m_imu_recovery_attempts = 0;
    m_preImuRecoveryState = m_currentState;
    
    // If we're balancing, use minimal interruption mode
    if (m_preImuRecoveryState == SystemState::BALANCING) {
        minimal_interruption = true;
    }
    
    // Configure recovery parameters based on current state
    int max_attempts = m_max_imu_recovery_attempts;
    int retry_delay_ms = m_preImuRecoveryState == SystemState::BALANCING ? 200 : 500;
    
    // Create recovery command with appropriate parameters
    IMU_AttemptRecovery cmd(minimal_interruption, max_attempts, retry_delay_ms);
    m_eventBus.publish(cmd);
    
    // Update recovery attempt counter
    m_imu_recovery_attempts++;
    
    // Start monitoring the IMU state to detect if it gets stuck
    monitorIMUState();
}

void StateManager::initiateCalibration(bool force) {
    ESP_LOGI(TAG, "Initiating calibration (force: %s)", force ? "true" : "false");
    
    // Check if we can calibrate from the current state
    if (m_currentState == SystemState::IDLE || force) {
        // We can calibrate from IDLE state
        // Don't change system state - just request calibration
        IMU_CalibrationRequest req_event;
        m_eventBus.publish(req_event);
        m_pending_calibration = false;
    } else if (m_currentState == SystemState::BALANCING && !force) {
        // Can't calibrate while balancing, set pending flag and stop
        ESP_LOGW(TAG, "Cannot calibrate while BALANCING. Setting pending calibration flag.");
        m_pending_calibration = true;
        
        // Publish calibration rejected event
        IMU_CalibrationRequestRejected reject_event(IMU_CalibrationRequestRejected::Reason::NOT_IDLE, true);
        m_eventBus.publish(reject_event);
    } else {
        // Other states (FALLEN, FATAL_ERROR, etc.)
        ESP_LOGW(TAG, "Cannot calibrate from current state: %s", stateToString(m_currentState).c_str());
        
        // Publish calibration rejected event
        IMU_CalibrationRequestRejected reject_event(IMU_CalibrationRequestRejected::Reason::OTHER, false);
        m_eventBus.publish(reject_event);
    }
}

void StateManager::requestImuRecovery() {
    // Replaced by initiateIMURecovery, but kept for backward compatibility
    initiateIMURecovery(false);
}

void StateManager::monitorIMUState() {
    // This function would implement a watchdog to detect if IMU gets stuck in RECOVERY state
    // Could be implemented using a timer or by checking time since recovery started
    
    // For now, just a placeholder - actual implementation would need a timer
    ESP_LOGD(TAG, "IMU state monitoring started");
    
    // In a real implementation, we'd start a timer here that checks periodically
    // if the IMU has been in RECOVERY state too long, and if so, forces a transition
    // to FATAL_ERROR or another appropriate state.
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
    
    // Request another recovery attempt with delay
    initiateIMURecovery(m_currentState == SystemState::BALANCING);
}

void StateManager::handleEnableRecovery(const UI_EnableFallRecovery& event) {
    setAutoRecovery(true);
}

void StateManager::handleDisableRecovery(const UI_DisableFallRecovery& event) {
    setAutoRecovery(false);
    if (m_withinRecoveryAngle) {
        m_withinRecoveryAngle = false;
        m_recoveryAngleStartTimeUs = 0;
        ESP_LOGD(TAG,"Auto-recovery disabled, resetting active recovery timer.");
    }
}

void StateManager::handleEnableFallDetect(const UI_EnableFallDetection& event) {
    enableFallDetection(true);
}

void StateManager::handleDisableFallDetect(const UI_DisableFallDetection& event) {
    enableFallDetection(false);
}

void StateManager::handleCalibrationRejected(const IMU_CalibrationRequestRejected& event) {
    ESP_LOGW(TAG, "Calibration was rejected: %s", 
             event.reason == IMU_CalibrationRequestRejected::Reason::NOT_IDLE ? "Not in IDLE state" :
             event.reason == IMU_CalibrationRequestRejected::Reason::RECOVERY_IN_PROGRESS ? "Recovery in progress" :
             "Other reason");
    
    // If retry is requested, set the pending flag
    if (event.retryWhenPossible) {
        ESP_LOGI(TAG, "Calibration will be retried when possible");
        m_pending_calibration = true;
    }
}