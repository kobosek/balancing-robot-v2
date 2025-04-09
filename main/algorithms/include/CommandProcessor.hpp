#pragma once
#include "EventBus.hpp"
#include "SystemState.hpp"
#include "SystemStateChangedEvent.hpp"
#include "TargetMovementCommand.hpp" // Output event (now contains pitch offset)
#include "JoystickInputEvent.hpp"    // Input event
#include "ConfigData.hpp"           // For ControlConfig
#include "esp_log.h"
#include <mutex>                    // For thread safety
#include "esp_timer.h"              // For timer

// Forward declaration
class ConfigurationService;

class CommandProcessor {
public:
    CommandProcessor(EventBus& bus, ConfigurationService& configService);
    ~CommandProcessor();

    esp_err_t init(); // Subscribe to events, load config, etc.


private:
    static constexpr const char* TAG = "CommandProc";
    static constexpr float JOYSTICK_DEADZONE = 0.10f; // Ignore small inputs
    static constexpr uint64_t INPUT_TIMEOUT_US = 500 * 1000; // 500ms timeout
    static constexpr uint64_t TIMEOUT_CHECK_INTERVAL_US = 100 * 1000; // Check every 100ms

    // Max speeds/offsets (Loaded from config)
    // float m_max_linear_velocity = 0.3f; // <<< REMOVED
    float m_max_target_pitch_offset_deg = 5.0f; // <<< ADDED (Loaded from config)
    float m_max_angular_velocity_dps = 60.0f; // Max dps from joystick X
    float m_joystick_exponent = 1.5f;       // (Loaded from config)

    EventBus& m_eventBus;
    ConfigurationService& m_configService;
    SystemState m_current_state = SystemState::INIT;

    // --- Target State (Protected) ---
    float m_target_pitch_offset_deg = 0.0f;   // <<< RENAMED/REPURPOSED from linVel
    float m_target_angular_velocity_dps = 0.0f; // dps
    mutable std::mutex m_target_mutex;        // Mutex

    // --- Timeout Tracking ---
    int64_t m_last_input_time_us = 0;
    bool m_input_timed_out = true;
    esp_timer_handle_t m_timeout_timer = nullptr;

    // Event Handlers
    void handleSystemStateChange(const SystemStateChangedEvent& event);
    void handleJoystickInput(const JoystickInputEvent& event);

    // Internal Helpers
    void periodicTimeoutCheck();
    void publishTargetCommand(float pitchOffsetDeg, float angVelDps); // <<< MODIFIED signature
    void loadConfigParameters();
    esp_err_t startTimeoutTimer();
    esp_err_t stopTimeoutTimer();
    static void timeout_timer_callback(void* arg);
};