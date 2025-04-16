#include "CommandProcessor.hpp"
#include "EventTypes.hpp"
#include "SystemStateChangedEvent.hpp"
#include "TargetMovementCommand.hpp"
#include "SystemState.hpp"
#include "EventBus.hpp"
#include "BaseEvent.hpp"
#include "JoystickInputEvent.hpp" // Include correct input event
#include "ConfigurationService.hpp"
#include "ConfigData.hpp"
#include "esp_log.h"
#include "esp_check.h"
#include <algorithm>
#include <cmath>
#include "esp_timer.h" // Include for timer functions

static const char* TAG = "CommandProc"; // Moved TAG definition here

CommandProcessor::CommandProcessor(EventBus& bus, ConfigurationService& configService) :
    m_eventBus(bus),
    m_configService(configService),
    m_current_state(SystemState::INIT),
    m_target_pitch_offset_deg(0.0f), // <<< RENAMED/REPURPOSED
    m_target_angular_velocity_dps(0.0f),
    m_last_input_time_us(0),
    m_input_timed_out(true),
    m_timeout_timer(nullptr)
{

}

CommandProcessor::~CommandProcessor() {
    stopTimeoutTimer(); // Ensure timer is stopped and deleted
}

esp_err_t CommandProcessor::init() {
    ESP_LOGI(TAG, "Initializing Command Processor...");

    // Load initial config values
    loadConfigParameters();

    // <<< REMOVED: Subscriptions moved >>>

    m_last_input_time_us = esp_timer_get_time(); // Initialize timestamp
    m_input_timed_out = true; // Start in timed-out state

    // Create the timeout timer
    const esp_timer_create_args_t timer_args = {
            .callback = &CommandProcessor::timeout_timer_callback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK, // Can run in timer task
            .name = "cp_timeout_timer",
    };

    esp_err_t ret = esp_timer_create(&timer_args, &m_timeout_timer);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to create timeout timer");

    ESP_LOGI(TAG, "Command Processor Initialized.");
    return ESP_OK;
}

void CommandProcessor::subscribeToEvents(EventBus& bus) {
    // Subscribe to state changes
    bus.subscribe(EventType::SYSTEM_STATE_CHANGED, [this](const BaseEvent& ev){
        if(ev.type == EventType::SYSTEM_STATE_CHANGED) {
             this->handleSystemStateChange(static_cast<const SystemStateChangedEvent&>(ev));
        }
    });
    ESP_LOGI(TAG, "Subscribed to SYSTEM_STATE_CHANGED events.");
    // Subscribe to raw joystick input
    bus.subscribe(EventType::JOYSTICK_INPUT_RECEIVED, [this](const BaseEvent& ev){
        if(ev.type == EventType::JOYSTICK_INPUT_RECEIVED) {
            this->handleJoystickInput(static_cast<const JoystickInputEvent&>(ev));
        }
    });
    ESP_LOGI(TAG, "Subscribed to JOYSTICK_INPUT_RECEIVED events.");
    // Subscribe to config updates to reload parameters
    bus.subscribe(EventType::CONFIG_UPDATED, [this](const BaseEvent& ev) {
        ESP_LOGD(TAG, "Config update event received, reloading parameters.");
        this->loadConfigParameters();
    });
    ESP_LOGI(TAG, "Subscribed to CONFIG_UPDATED events.");
}


void CommandProcessor::loadConfigParameters() {
    ControlConfig controlConf = m_configService.getControlConfig();
    m_joystick_exponent = controlConf.joystick_exponent;
    m_max_target_pitch_offset_deg = controlConf.max_target_pitch_offset_deg; // <<< LOADED
    // TODO: Load max angular velocity if it becomes configurable
    ESP_LOGI(TAG, "Loaded Control Params: JoyExp=%.2f, MaxPitchOffset=%.2f deg", m_joystick_exponent, m_max_target_pitch_offset_deg);
}

void CommandProcessor::handleSystemStateChange(const SystemStateChangedEvent& event) {
    SystemState previous_state = event.previousState;
    m_current_state = event.newState;

    // --- Stop/Start Timer based on Balancing State ---
    if (m_current_state == SystemState::BALANCING && previous_state != SystemState::BALANCING) {
        ESP_LOGI(TAG, "CP: Entering BALANCING, resetting targets, starting timeout timer.");
        {
            std::lock_guard<std::mutex> lock(m_target_mutex);
            m_target_pitch_offset_deg = 0.0f; // <<< RESET
            m_target_angular_velocity_dps = 0.0f;
            m_last_input_time_us = esp_timer_get_time(); // Reset timer on enter
            m_input_timed_out = true; // Start timed out, requires first command
        }
        // Publish zero command to ensure algorithm starts from zero target
        publishTargetCommand(0.0f, 0.0f); // Pitch Offset 0, Angular Vel 0
        startTimeoutTimer(); // Start the periodic check

    } else if (m_current_state != SystemState::BALANCING && previous_state == SystemState::BALANCING) {
        ESP_LOGI(TAG, "CP: Leaving BALANCING, stopping timeout timer, resetting targets.");
        stopTimeoutTimer(); // Stop the periodic check
        bool had_velocity = false;
        {
            std::lock_guard<std::mutex> lock(m_target_mutex);
            // Only reset and publish if targets were non-zero
            if (std::fabs(m_target_pitch_offset_deg) > 1e-4f || std::fabs(m_target_angular_velocity_dps) > 1e-4f) { // <<< CHECK PITCH OFFSET
                 m_target_pitch_offset_deg = 0.0f; // <<< RESET PITCH OFFSET
                 m_target_angular_velocity_dps = 0.0f;
                 had_velocity = true;
            }
            m_input_timed_out = true; // Ensure timed out state when not balancing
        }
         // Publish zero command if we stopped moving
        if (had_velocity) {
            publishTargetCommand(0.0f, 0.0f); // Pitch Offset 0, Angular Vel 0
        }
    }
}

void CommandProcessor::handleJoystickInput(const JoystickInputEvent& event) {
    int64_t current_time = esp_timer_get_time();
    bool was_timed_out = false;

    { // Lock scope for updating timestamp and timeout flag
        std::lock_guard<std::mutex> lock(m_target_mutex);
        m_last_input_time_us = current_time;
        was_timed_out = m_input_timed_out; // Check if we *were* timed out
        m_input_timed_out = false; // Input is now active
    }

    if (m_current_state != SystemState::BALANCING) {
        // ESP_LOGV(TAG, "Ignoring joystick input, not in BALANCING state.");
        return; // Only process input when balancing
    }

    // Apply Deadzone
    float effective_x = (std::fabs(event.x) < JOYSTICK_DEADZONE) ? 0.0f : event.x;
    float effective_y = (std::fabs(event.y) < JOYSTICK_DEADZONE) ? 0.0f : event.y;

    // --- Map joystick axes to robot control parameters ---
    // Y-axis controls target pitch angle offset (forward/backward tilt) <<< MODIFIED
    float mapped_y = std::copysign(std::pow(std::fabs(effective_y), m_joystick_exponent), effective_y);
    // Negative Y typically means forward on joystick -> positive pitch offset
    float desiredPitchOffset_deg = m_max_target_pitch_offset_deg * (-mapped_y);

    // X-axis controls angular velocity (turning)
    float mapped_x = std::copysign(std::pow(std::fabs(effective_x), m_joystick_exponent), effective_x);
    float desiredAngVelDps = m_max_angular_velocity_dps * (-mapped_x); // Negative X might mean turn right

    // --- Check if targets changed OR if input was previously timed out ---
    bool needs_publish = false;
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        if (was_timed_out || // Always publish if recovering from timeout
            std::fabs(desiredPitchOffset_deg - m_target_pitch_offset_deg) > 1e-4f || // <<< CHECK PITCH OFFSET
            std::fabs(desiredAngVelDps - m_target_angular_velocity_dps) > 1e-4f)
        {
            m_target_pitch_offset_deg = desiredPitchOffset_deg; // <<< STORE PITCH OFFSET
            m_target_angular_velocity_dps = desiredAngVelDps;
            needs_publish = true;
        }
    } // Mutex released

    if (needs_publish) {
        publishTargetCommand(desiredPitchOffset_deg, desiredAngVelDps); // <<< PASS PITCH OFFSET
    }
}

void CommandProcessor::periodicTimeoutCheck() {
    if (m_current_state != SystemState::BALANCING) {
        return; // Should have been stopped by state change, but double-check
    }

    bool publish_zero = false;
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        // Check if already timed out to avoid repeated logs/publishes
        if (!m_input_timed_out && (esp_timer_get_time() - m_last_input_time_us) > INPUT_TIMEOUT_US) {
            ESP_LOGW(TAG, "CP: Joystick input timeout detected by periodic check! Setting targets to zero.");
            // Check if targets were actually non-zero before resetting
            if (std::fabs(m_target_pitch_offset_deg) > 1e-4f || std::fabs(m_target_angular_velocity_dps) > 1e-4f) { // <<< CHECK PITCH OFFSET
                m_target_pitch_offset_deg = 0.0f; // <<< RESET PITCH OFFSET
                m_target_angular_velocity_dps = 0.0f;
                publish_zero = true; // Need to publish the zero command
            }
            m_input_timed_out = true; // Mark as timed out
        }
    } // Mutex released

    // Publish zero command outside the lock if needed
    if (publish_zero) {
        publishTargetCommand(0.0f, 0.0f); // Pitch Offset 0, Angular Vel 0
    }
}

// Static timer callback remains the same
void CommandProcessor::timeout_timer_callback(void* arg) {
    CommandProcessor* instance = static_cast<CommandProcessor*>(arg);
    if (instance) {
        instance->periodicTimeoutCheck();
    }
}

// Timer start/stop helpers remain the same
esp_err_t CommandProcessor::startTimeoutTimer() {
    if (!m_timeout_timer) {
        ESP_LOGE(TAG, "Timeout timer handle is null!");
        return ESP_ERR_INVALID_STATE;
    }
    if (esp_timer_is_active(m_timeout_timer)) {
        ESP_LOGD(TAG, "Timeout timer already active.");
        return ESP_OK;
    }
    esp_err_t ret = esp_timer_start_periodic(m_timeout_timer, TIMEOUT_CHECK_INTERVAL_US);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Started periodic timeout timer (%llu us interval).", TIMEOUT_CHECK_INTERVAL_US);
    } else {
        ESP_LOGE(TAG, "Failed to start timeout timer: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t CommandProcessor::stopTimeoutTimer() {
    if (!m_timeout_timer) { return ESP_OK; }
    if (!esp_timer_is_active(m_timeout_timer)) { return ESP_OK; }
    esp_err_t ret = esp_timer_stop(m_timeout_timer);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Stopped periodic timeout timer.");
    } else {
        ESP_LOGE(TAG, "Failed to stop timeout timer: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Helper to publish the final target command <<< MODIFIED SIGNATURE >>>
void CommandProcessor::publishTargetCommand(float pitchOffsetDeg, float angVelDps) {
    TargetMovementCommand cmd(pitchOffsetDeg, angVelDps);
    m_eventBus.publish(cmd);
    ESP_LOGD(TAG, "CP: Published Target CMD: PitchOffset=%.2f deg, AngVel=%.2f dps", pitchOffsetDeg, angVelDps);
}