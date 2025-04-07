// main/algorithms/CommandProcessor.cpp
#include "CommandProcessor.hpp"
#include "EventTypes.hpp"
#include "SystemStateChangedEvent.hpp"
#include "TargetMovementCommand.hpp"
#include "SystemState.hpp"
#include "EventBus.hpp"
#include "BaseEvent.hpp"
#include "JoystickInputEvent.hpp" // <<<--- Include correct input event
#include "esp_log.h"
#include <algorithm>
#include <cmath>
#include "esp_timer.h" // <<<--- Include for timer functions

CommandProcessor::CommandProcessor(EventBus& bus) :
    m_eventBus(bus),
    m_current_state(SystemState::INIT),
    m_target_linear_velocity(0.0f),
    m_target_angular_velocity(0.0f)
    // m_target_mutex default constructed
    // m_last_input_time_us = 0
    // m_input_timed_out = true
{
    // TODO: Load max velocities from ConfigurationService if desired
}

esp_err_t CommandProcessor::init() {
    ESP_LOGI(TAG, "Initializing Command Processor...");

    // Subscribe to state changes to reset targets
    m_eventBus.subscribe(EventType::SYSTEM_STATE_CHANGED, [this](const BaseEvent& ev){
        if(ev.type == EventType::SYSTEM_STATE_CHANGED) {
             this->handleSystemStateChange(static_cast<const SystemStateChangedEvent&>(ev));
        }
    });

    // Subscribe to raw joystick input
    m_eventBus.subscribe(EventType::JOYSTICK_INPUT_RECEIVED, [this](const BaseEvent& ev){
        if(ev.type == EventType::JOYSTICK_INPUT_RECEIVED) {
            this->handleJoystickInput(static_cast<const JoystickInputEvent&>(ev));
        }
    });
    ESP_LOGI(TAG, "Subscribed to JOYSTICK_INPUT_RECEIVED events.");

    m_last_input_time_us = esp_timer_get_time(); // Initialize timestamp
    m_input_timed_out = true; // Start in timed-out state

    ESP_LOGI(TAG, "Command Processor Initialized.");
    return ESP_OK;
}

// --- Thread-safe Getters ---
float CommandProcessor::getTargetLinearVelocity() const {
    std::lock_guard<std::mutex> lock(m_target_mutex);
    // Check timeout before returning (needs non-const access)
    const_cast<CommandProcessor*>(this)->checkInputTimeout();
    return m_target_linear_velocity;
}

float CommandProcessor::getTargetAngularVelocity() const {
    std::lock_guard<std::mutex> lock(m_target_mutex);
    // Check timeout before returning
    const_cast<CommandProcessor*>(this)->checkInputTimeout();
    return m_target_angular_velocity;
}
// --- End Getters ---


void CommandProcessor::handleSystemStateChange(const SystemStateChangedEvent& event) {
    ESP_LOGD(TAG, "CP: State changed from %d to %d", static_cast<int>(event.previousState), static_cast<int>(event.newState));
    SystemState previous_state = event.previousState;
    m_current_state = event.newState;

    // --- Reset Logic ---
    // If leaving BALANCING state
    if (previous_state == SystemState::BALANCING && m_current_state != SystemState::BALANCING) {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        // Only reset and publish if targets were non-zero
        if (std::fabs(m_target_linear_velocity) > 1e-4f || std::fabs(m_target_angular_velocity) > 1e-4f) {
             ESP_LOGI(TAG, "CP: Leaving BALANCING, resetting targets and publishing zero.");
             m_target_linear_velocity = 0.0f;
             m_target_angular_velocity = 0.0f;
             m_input_timed_out = true; // Ensure timed out state
             // Unlock mutex *before* publishing event if possible, or just publish here
             TargetMovementCommand zeroCmd(0.0f, 0.0f);
             m_eventBus.publish(zeroCmd);
        } else {
            // Ensure timeout flag is set even if targets were already zero
             m_input_timed_out = true;
        }
    }
    // If entering BALANCING state
    else if (previous_state != SystemState::BALANCING && m_current_state == SystemState::BALANCING) {
         ESP_LOGI(TAG, "CP: Entering BALANCING, resetting targets to zero and timeout state.");
         std::lock_guard<std::mutex> lock(m_target_mutex);
         m_target_linear_velocity = 0.0f;
         m_target_angular_velocity = 0.0f;
         m_last_input_time_us = esp_timer_get_time(); // Reset timer
         m_input_timed_out = true; // Start timed out, requires first command
         // Publish zero command to ensure algorithm starts from zero target
         TargetMovementCommand zeroCmd(0.0f, 0.0f);
         m_eventBus.publish(zeroCmd);
    }
}


void CommandProcessor::handleJoystickInput(const JoystickInputEvent& event) {
    // Update timestamp regardless of state
    m_last_input_time_us = esp_timer_get_time();

    if (m_current_state != SystemState::BALANCING) {
        // ESP_LOGV(TAG, "Ignoring joystick input, not in BALANCING state.");
        return; // Only process input when balancing
    }

    // Apply Deadzone
    float effective_x = (std::fabs(event.x) < JOYSTICK_DEADZONE) ? 0.0f : event.x;
    float effective_y = (std::fabs(event.y) < JOYSTICK_DEADZONE) ? 0.0f : event.y;

    // --- Map joystick axes to robot velocities ---
    // Y-axis controls linear velocity (forward/backward)
    float desiredLinVel = m_max_linear_velocity * effective_y;
    // X-axis controls angular velocity (turning)
    // Negative X might mean turn right, depending on coordinate system/preference
    float desiredAngVelDps = m_max_angular_velocity_dps * (-effective_x);

    // --- Check if targets changed OR if input timed out ---
    bool needs_publish = false;
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        if (m_input_timed_out || // Always publish if recovering from timeout
            std::fabs(desiredLinVel - m_target_linear_velocity) > 1e-4f ||
            std::fabs(desiredAngVelDps - m_target_angular_velocity) > 1e-4f)
        {
            m_target_linear_velocity = desiredLinVel;
            m_target_angular_velocity = desiredAngVelDps;
            m_input_timed_out = false; // Input is now active
            needs_publish = true;
        }
    } // Mutex released

    if (needs_publish) {
        publishTargetCommand(desiredLinVel, desiredAngVelDps);
    }
}

// Check for input timeout - called periodically or before getting targets
void CommandProcessor::checkInputTimeout() {
    // No timeout check needed if not balancing or already timed out
    if (m_current_state != SystemState::BALANCING || m_input_timed_out) {
        return;
    }

    if ((esp_timer_get_time() - m_last_input_time_us) > INPUT_TIMEOUT_US) {
        ESP_LOGW(TAG, "CP: Joystick input timeout! Setting targets to zero.");
        bool had_velocity = false;
        {
            std::lock_guard<std::mutex> lock(m_target_mutex);
            // Check if targets were actually non-zero before resetting
            if (std::fabs(m_target_linear_velocity) > 1e-4f || std::fabs(m_target_angular_velocity) > 1e-4f) {
                m_target_linear_velocity = 0.0f;
                m_target_angular_velocity = 0.0f;
                had_velocity = true;
            }
            m_input_timed_out = true; // Mark as timed out
        } // Mutex released

        // Publish zero command only if we actually stopped moving
        if (had_velocity) {
            publishTargetCommand(0.0f, 0.0f);
        }
    }
}

// Helper to publish the final target command
void CommandProcessor::publishTargetCommand(float linVel, float angVelDps) {
    TargetMovementCommand cmd(linVel, angVelDps);
    m_eventBus.publish(cmd);
    ESP_LOGD(TAG, "CP: Published Target CMD: Lin=%.3f m/s, Ang=%.2f dps", linVel, angVelDps);
}