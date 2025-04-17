// ================================================
// File: main/algorithms/CommandProcessor.cpp
// ================================================
#include "CommandProcessor.hpp"
#include "EventTypes.hpp"
#include "SystemStateChangedEvent.hpp"
#include "TargetMovementCommand.hpp"
#include "SystemState.hpp"
#include "EventBus.hpp"
#include "BaseEvent.hpp"
#include "JoystickInputEvent.hpp"
#include "ConfigUpdatedEvent.hpp" // Include event with payload
#include "ConfigData.hpp"
#include "esp_log.h"
#include "esp_check.h"
#include <algorithm>
#include <cmath>
#include "esp_timer.h"

static const char* TAG = "CommandProc";

// --- Constructor: Takes initial config structs ---
CommandProcessor::CommandProcessor(EventBus& bus, const ControlConfig& initialControl, const SystemBehaviorConfig& initialBehavior) :
    m_eventBus(bus),                                // 1
    // m_configService removed                   // 2
    m_current_state(SystemState::INIT),             // 3
    m_target_pitch_offset_deg(0.0f),                // 4
    m_target_angular_velocity_dps(0.0f),            // 5
    // m_target_mutex default initialized          // 6
    m_last_input_time_us(0),                        // 7
    m_input_timed_out(true),                        // 8
    m_timeout_timer(nullptr),                       // 9
    // Configurable parameters initialized by applyConfig later
    m_joystick_exponent(1.5f),                      // 10 (Default)
    m_max_target_pitch_offset_deg(5.0f),            // 11 (Default)
    m_joystick_deadzone(0.1f),                      // 12 (Default)
    m_input_timeout_us(500000),                     // 13 (Default)
    m_timeout_check_interval_us(100000),            // 14 (Default)
    m_max_angular_velocity_dps(60.0f)               // 15 (Default)
{
    ESP_LOGI(TAG, "Command Processor constructed.");
    applyConfig(initialControl, initialBehavior); // Apply initial config
}

CommandProcessor::~CommandProcessor() {
    stopTimeoutTimer(); // Ensure timer is stopped and deleted
}


esp_err_t CommandProcessor::init() {
    ESP_LOGI(TAG, "Initializing Command Processor...");

    // Config is applied in constructor
    // loadConfigParameters();

    m_last_input_time_us = esp_timer_get_time(); // Initialize timestamp
    m_input_timed_out = true; // Start in timed-out state

    // Create the timeout timer
    const esp_timer_create_args_t timer_args = {
            .callback = &CommandProcessor::timeout_timer_callback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "cp_timeout_timer",
            .skip_unhandled_events = false // Initialize this field
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
        this->handleConfigUpdate(ev); // Call specific handler
    });
    ESP_LOGI(TAG, "Subscribed to CONFIG_UPDATED events.");
}

// Helper to apply config values
void CommandProcessor::applyConfig(const ControlConfig& controlConf, const SystemBehaviorConfig& behaviorConf) {
    m_joystick_exponent = controlConf.joystick_exponent;
    m_max_target_pitch_offset_deg = controlConf.max_target_pitch_offset_deg;
    m_joystick_deadzone = behaviorConf.joystick_deadzone;
    m_input_timeout_us = behaviorConf.joystick_timeout_ms * 1000ULL; // Convert ms to us
    m_timeout_check_interval_us = behaviorConf.joystick_check_interval_ms * 1000ULL; // Convert ms to us
    m_max_angular_velocity_dps = behaviorConf.max_target_angular_velocity_dps;

    ESP_LOGI(TAG, "Applied Control Params: JoyExp=%.2f, MaxPitchOffset=%.2f deg, MaxAngVel=%.1f dps, Deadzone=%.2f, Timeout=%lluus, CheckInt=%lluus",
             m_joystick_exponent, m_max_target_pitch_offset_deg, m_max_angular_velocity_dps, m_joystick_deadzone, m_input_timeout_us, m_timeout_check_interval_us);

    // If timer is running, update its period (requires stopping and starting)
    if (m_timeout_timer && esp_timer_is_active(m_timeout_timer)) {
        ESP_LOGI(TAG, "Updating active timeout timer interval to %llu us", m_timeout_check_interval_us);
        // Need to stop before starting with new period
        esp_err_t stop_ret = stopTimeoutTimer();
        if (stop_ret == ESP_OK) {
            startTimeoutTimer(); // Will use the new interval
        } else {
            ESP_LOGE(TAG, "Failed to stop timer to update interval, timer may have old interval!");
        }
    }
}

// Handle config update event
void CommandProcessor::handleConfigUpdate(const BaseEvent& event) {
    if (event.type != EventType::CONFIG_UPDATED) return;
    ESP_LOGD(TAG, "Config update event received, applying new parameters.");
    const auto& configEvent = static_cast<const ConfigUpdatedEvent&>(event);
    // Extract relevant parts and apply
    applyConfig(configEvent.configData.control, configEvent.configData.behavior);
}

void CommandProcessor::handleSystemStateChange(const SystemStateChangedEvent& event) {
    SystemState previous_state = event.previousState;
    m_current_state = event.newState;

    // --- Stop/Start Timer based on Balancing State ---
    if (m_current_state == SystemState::BALANCING && previous_state != SystemState::BALANCING) {
        ESP_LOGI(TAG, "CP: Entering BALANCING, resetting targets, starting timeout timer.");
        {
            std::lock_guard<std::mutex> lock(m_target_mutex);
            m_target_pitch_offset_deg = 0.0f;
            m_target_angular_velocity_dps = 0.0f;
            m_last_input_time_us = esp_timer_get_time();
            m_input_timed_out = true;
        }
        publishTargetCommand(0.0f, 0.0f);
        startTimeoutTimer();

    } else if (m_current_state != SystemState::BALANCING && previous_state == SystemState::BALANCING) {
        ESP_LOGI(TAG, "CP: Leaving BALANCING, stopping timeout timer, resetting targets.");
        stopTimeoutTimer();
        bool had_velocity = false;
        {
            std::lock_guard<std::mutex> lock(m_target_mutex);
            if (std::fabs(m_target_pitch_offset_deg) > 1e-4f || std::fabs(m_target_angular_velocity_dps) > 1e-4f) {
                 m_target_pitch_offset_deg = 0.0f;
                 m_target_angular_velocity_dps = 0.0f;
                 had_velocity = true;
            }
            m_input_timed_out = true;
        }
        if (had_velocity) {
            publishTargetCommand(0.0f, 0.0f);
        }
    }
}

void CommandProcessor::handleJoystickInput(const JoystickInputEvent& event) {
    int64_t current_time = esp_timer_get_time();
    bool was_timed_out = false;

    { // Lock scope for updating timestamp and timeout flag
        std::lock_guard<std::mutex> lock(m_target_mutex);
        m_last_input_time_us = current_time;
        was_timed_out = m_input_timed_out;
        m_input_timed_out = false;
    }

    if (m_current_state != SystemState::BALANCING) {
        return; // Only process input when balancing
    }

    // Apply Deadzone (using member variable)
    float effective_x = (std::fabs(event.x) < m_joystick_deadzone) ? 0.0f : event.x;
    float effective_y = (std::fabs(event.y) < m_joystick_deadzone) ? 0.0f : event.y;

    // --- Map joystick axes to robot control parameters ---
    float mapped_y = std::copysign(std::pow(std::fabs(effective_y), m_joystick_exponent), effective_y);
    float desiredPitchOffset_deg = m_max_target_pitch_offset_deg * (-mapped_y);

    float mapped_x = std::copysign(std::pow(std::fabs(effective_x), m_joystick_exponent), effective_x);
    float desiredAngVelDps = m_max_angular_velocity_dps * (-mapped_x);

    // --- Check if targets changed OR if input was previously timed out ---
    bool needs_publish = false;
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        if (was_timed_out ||
            std::fabs(desiredPitchOffset_deg - m_target_pitch_offset_deg) > 1e-4f ||
            std::fabs(desiredAngVelDps - m_target_angular_velocity_dps) > 1e-4f)
        {
            m_target_pitch_offset_deg = desiredPitchOffset_deg;
            m_target_angular_velocity_dps = desiredAngVelDps;
            needs_publish = true;
        }
    } // Mutex released

    if (needs_publish) {
        publishTargetCommand(desiredPitchOffset_deg, desiredAngVelDps);
    }
}

void CommandProcessor::periodicTimeoutCheck() {
    if (m_current_state != SystemState::BALANCING) {
        return;
    }

    bool publish_zero = false;
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        // Use member variable for timeout
        if (!m_input_timed_out && (esp_timer_get_time() - m_last_input_time_us) > m_input_timeout_us) {
            ESP_LOGW(TAG, "CP: Joystick input timeout detected by periodic check! Setting targets to zero.");
            if (std::fabs(m_target_pitch_offset_deg) > 1e-4f || std::fabs(m_target_angular_velocity_dps) > 1e-4f) {
                m_target_pitch_offset_deg = 0.0f;
                m_target_angular_velocity_dps = 0.0f;
                publish_zero = true;
            }
            m_input_timed_out = true;
        }
    } // Mutex released

    if (publish_zero) {
        publishTargetCommand(0.0f, 0.0f);
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
    // Use member variable for interval
    esp_err_t ret = esp_timer_start_periodic(m_timeout_timer, m_timeout_check_interval_us);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Started periodic timeout timer (%llu us interval).", m_timeout_check_interval_us);
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

// Helper to publish the final target command
void CommandProcessor::publishTargetCommand(float pitchOffsetDeg, float angVelDps) {
    TargetMovementCommand cmd(pitchOffsetDeg, angVelDps);
    m_eventBus.publish(cmd);
    ESP_LOGD(TAG, "CP: Published Target CMD: PitchOffset=%.2f deg, AngVel=%.2f dps", pitchOffsetDeg, angVelDps);
}