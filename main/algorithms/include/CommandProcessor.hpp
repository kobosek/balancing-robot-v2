// ================================================
// File: main/algorithms/include/CommandProcessor.hpp
// ================================================
#pragma once
#include "EventBus.hpp"
#include "SystemState.hpp"
#include "SystemStateChangedEvent.hpp"
#include "TargetMovementCommand.hpp" // Output event (now contains pitch offset)
#include "JoystickInputEvent.hpp"    // Input event
#include "ConfigData.hpp"           // For ControlConfig & SystemBehaviorConfig
#include "esp_log.h"
#include <mutex>                    // For thread safety
#include "esp_timer.h"              // For timer

// Forward declarations
// class ConfigurationService; // REMOVE
class BaseEvent; // ADD
class ConfigUpdatedEvent; // ADD

class CommandProcessor {
public:
    // Constructor takes initial config structs
    CommandProcessor(EventBus& bus, const ControlConfig& initialControl, const SystemBehaviorConfig& initialBehavior);
    ~CommandProcessor();

    esp_err_t init(); // Subscribe to events, etc.
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "CommandProc";

    // --- Dependencies ---
    EventBus& m_eventBus;                       // Declaration Order: 1
    // ConfigurationService& m_configService;      // REMOVE

    // --- Internal State ---
    SystemState m_current_state;                // Declaration Order: 3
    float m_target_pitch_offset_deg;            // Declaration Order: 4
    float m_target_angular_velocity_dps;        // Declaration Order: 5
    mutable std::mutex m_target_mutex;          // Declaration Order: 6

    // --- Timeout Tracking ---
    int64_t m_last_input_time_us;               // Declaration Order: 7
    bool m_input_timed_out;                     // Declaration Order: 8
    esp_timer_handle_t m_timeout_timer;         // Declaration Order: 9

    // --- Configurable Parameters (Store locally) ---
    float m_joystick_exponent;                  // Declaration Order: 10 (From ControlConfig)
    float m_max_target_pitch_offset_deg;        // Declaration Order: 11 (From ControlConfig)
    float m_joystick_deadzone;                  // Declaration Order: 12 (From SystemBehaviorConfig)
    uint64_t m_input_timeout_us;                // Declaration Order: 13 (From SystemBehaviorConfig)
    uint64_t m_timeout_check_interval_us;       // Declaration Order: 14 (From SystemBehaviorConfig)
    float m_max_angular_velocity_dps;           // Declaration Order: 15 (From SystemBehaviorConfig)


    // --- Event Handlers ---
    void handleSystemStateChange(const SystemStateChangedEvent& event);
    void handleJoystickInput(const JoystickInputEvent& event);
    void handleConfigUpdate(const BaseEvent& event); // ADD

    // --- Internal Helpers ---
    void periodicTimeoutCheck();
    void publishTargetCommand(float pitchOffsetDeg, float angVelDps);
    // void loadConfigParameters(); // REMOVE or make private apply
    void applyConfig(const ControlConfig& controlConf, const SystemBehaviorConfig& behaviorConf); // ADD
    esp_err_t startTimeoutTimer();
    esp_err_t stopTimeoutTimer();
    static void timeout_timer_callback(void* arg);
};