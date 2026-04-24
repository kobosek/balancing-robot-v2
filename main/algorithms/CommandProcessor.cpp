// ================================================
// File: main/algorithms/CommandProcessor.cpp
// ================================================
#include "CommandProcessor.hpp"
#include "COMMAND_InputModeChanged.hpp"
#include "MOTION_TargetMovement.hpp"
#include "EventBus.hpp"
#include "BaseEvent.hpp"
#include "UI_JoystickInput.hpp"
#include "CONFIG_FullConfigUpdate.hpp" // Include event with payload
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
    m_target_pitch_offset_deg(0.0f),                // 3
    m_target_angular_velocity_dps(0.0f),            // 4
    // m_target_mutex default initialized          // 5
    m_last_input_time_us(0),                        // 6
    m_accepting_input(false),                       // 7
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
    stopTimeoutTimer();
    if (m_timeout_timer) {
        esp_err_t ret = esp_timer_delete(m_timeout_timer);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete timeout timer: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Deleted timeout timer.");
            m_timeout_timer = nullptr;
        }
    }
}


esp_err_t CommandProcessor::init() {
    ESP_LOGI(TAG, "Initializing Command Processor...");

    // Config is applied in constructor
    // loadConfigParameters();

    m_last_input_time_us = esp_timer_get_time(); // Initialize timestamp
    m_input_timed_out = true; // Start in timed-out state

    // Create the timeout timer
    esp_timer_create_args_t timer_args = {};
    timer_args.callback = &CommandProcessor::timeout_timer_callback;
    timer_args.arg = this;
    timer_args.name = "joystick_timeout"; // Optional timer name

    ESP_LOGI(TAG, "Creating joystick timeout timer...");
    esp_err_t ret = esp_timer_create(&timer_args, &m_timeout_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timeout timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Command Processor Initialized.");
    return ESP_OK;
}

void CommandProcessor::handleEvent(const BaseEvent& event) {
    if (event.is<COMMAND_InputModeChanged>()) {
        handleInputModeChange(event.as<COMMAND_InputModeChanged>());
    } else if (event.is<UI_JoystickInput>()) {
        handleJoystickInput(event.as<UI_JoystickInput>());
    } else if (event.is<CONFIG_FullConfigUpdate>()) {
        handleConfigUpdate(event.as<CONFIG_FullConfigUpdate>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

// Helper to apply config values
void CommandProcessor::applyConfig(const ControlConfig& controlConf, const SystemBehaviorConfig& behaviorConf) {
    bool restart_timer = false;
    float joystick_exponent = controlConf.joystick_exponent;
    float max_target_pitch_offset_deg = controlConf.max_target_pitch_offset_deg;
    float joystick_deadzone = behaviorConf.joystick_deadzone;
    uint64_t input_timeout_us = behaviorConf.joystick_timeout_ms * 1000ULL;
    uint64_t timeout_check_interval_us = behaviorConf.joystick_check_interval_ms * 1000ULL;
    float max_angular_velocity_dps = behaviorConf.max_target_angular_velocity_dps;
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        m_joystick_exponent = joystick_exponent;
        m_max_target_pitch_offset_deg = max_target_pitch_offset_deg;
        m_joystick_deadzone = joystick_deadzone;
        m_input_timeout_us = input_timeout_us;
        m_timeout_check_interval_us = timeout_check_interval_us;
        m_max_angular_velocity_dps = max_angular_velocity_dps;
        restart_timer = m_timeout_timer && esp_timer_is_active(m_timeout_timer);
    }

    ESP_LOGI(TAG, "Applied Control Params: JoyExp=%.2f, MaxPitchOffset=%.2f deg, MaxAngVel=%.1f dps, Deadzone=%.2f, Timeout=%lluus, CheckInt=%lluus",
             joystick_exponent, max_target_pitch_offset_deg, max_angular_velocity_dps, joystick_deadzone, input_timeout_us, timeout_check_interval_us);

    // If timer is running, update its period (requires stopping and starting)
    if (restart_timer) {
        ESP_LOGI(TAG, "Updating active timeout timer interval to %llu us", timeout_check_interval_us);
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
void CommandProcessor::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
    // Process the full config update
    ESP_LOGI(TAG, "Processing config update.");
    
    // Apply the new config values from the event payload
    applyConfig(event.configData.control, event.configData.behavior);
}

void CommandProcessor::handleInputModeChange(const COMMAND_InputModeChanged& event) {
    bool was_accepting_input = false;
    const bool accepting_input = event.acceptingInput;
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        was_accepting_input = m_accepting_input;
        m_accepting_input = accepting_input;
    }

    // --- Stop/Start Timer based on Balancing State ---
    if (accepting_input && !was_accepting_input) {
        ESP_LOGI(TAG, "CP: Enabling command input, resetting targets, starting timeout timer.");
        {
            std::lock_guard<std::mutex> lock(m_target_mutex);
            m_target_pitch_offset_deg = 0.0f;
            m_target_angular_velocity_dps = 0.0f;
            m_last_input_time_us = esp_timer_get_time();
            m_input_timed_out = true;
        }
        publishTargetCommand(0.0f, 0.0f);
        startTimeoutTimer();

    } else if (!accepting_input && was_accepting_input) {
        ESP_LOGI(TAG, "CP: Disabling command input, stopping timeout timer, resetting targets.");
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

void CommandProcessor::handleJoystickInput(const UI_JoystickInput& event) {
    int64_t current_time = esp_timer_get_time();
    bool was_timed_out = false;
    bool accepting_input = false;
    float joystick_deadzone = 0.0f;
    float joystick_exponent = 1.0f;
    float max_target_pitch_offset_deg = 0.0f;
    float max_angular_velocity_dps = 0.0f;

    { // Lock scope for updating timestamp, timeout flag, and copying config
        std::lock_guard<std::mutex> lock(m_target_mutex);
        m_last_input_time_us = current_time;
        was_timed_out = m_input_timed_out;
        m_input_timed_out = false;
        accepting_input = m_accepting_input;
        joystick_deadzone = m_joystick_deadzone;
        joystick_exponent = m_joystick_exponent;
        max_target_pitch_offset_deg = m_max_target_pitch_offset_deg;
        max_angular_velocity_dps = m_max_angular_velocity_dps;
    }

    if (!accepting_input) {
        return; // Only process input when balancing
    }

    // Apply Deadzone (using member variable)
    float effective_x = (std::fabs(event.x) < joystick_deadzone) ? 0.0f : event.x;
    float effective_y = (std::fabs(event.y) < joystick_deadzone) ? 0.0f : event.y;

    // --- Map joystick axes to robot control parameters ---
    float mapped_y = std::copysign(std::pow(std::fabs(effective_y), joystick_exponent), effective_y);
    float desiredPitchOffset_deg = max_target_pitch_offset_deg * (-mapped_y);

    float mapped_x = std::copysign(std::pow(std::fabs(effective_x), joystick_exponent), effective_x);
    float desiredAngVelDps = max_angular_velocity_dps * (-mapped_x);

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
    bool publish_zero = false;
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        if (!m_accepting_input) {
            return;
        }
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
    uint64_t timeout_check_interval_us = 0;
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        timeout_check_interval_us = m_timeout_check_interval_us;
    }
    esp_err_t ret = esp_timer_start_periodic(m_timeout_timer, timeout_check_interval_us);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Started periodic timeout timer (%llu us interval).", timeout_check_interval_us);
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
    MOTION_TargetMovement cmd(pitchOffsetDeg, angVelDps);
    m_eventBus.publish(cmd);
    ESP_LOGD(TAG, "CP: Published Target CMD: PitchOffset=%.2f deg, AngVel=%.2f dps", pitchOffsetDeg, angVelDps);
}
