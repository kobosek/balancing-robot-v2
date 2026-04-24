// ================================================
// File: main/algorithms/include/CommandProcessor.hpp
// ================================================
#pragma once
#include "EventBus.hpp"
#include "EventHandler.hpp"
#include "config/ControlConfig.hpp"
#include "config/SystemBehaviorConfig.hpp"
#include <mutex>                    // For thread safety


// Forward declarations
class BaseEvent;
class COMMAND_InputModeChanged;
class CONFIG_FullConfigUpdate;
class MOTION_TargetMovement;
class UI_JoystickInput;

class CommandProcessor : public EventHandler {
public:
    // Constructor takes initial config structs
    CommandProcessor(EventBus& bus, const ControlConfig& initialControl, const SystemBehaviorConfig& initialBehavior);
    ~CommandProcessor();

    esp_err_t init(); // Initialize the processor

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return "CommandProcessor"; }

private:
    static constexpr const char* TAG = "CommandProc";

    // --- Dependencies ---
    EventBus& m_eventBus;                       // Declaration Order: 1

    // --- Internal State ---
    float m_target_pitch_offset_deg;            // Declaration Order: 3
    float m_target_angular_velocity_dps;        // Declaration Order: 4
    mutable std::mutex m_target_mutex;          // Declaration Order: 5

    // --- Timeout Tracking ---
    int64_t m_last_input_time_us;               // Declaration Order: 6
    bool m_accepting_input;                     // Declaration Order: 7
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
    void handleInputModeChange(const COMMAND_InputModeChanged& event);
    void handleJoystickInput(const UI_JoystickInput& event);
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);

    // --- Internal Helpers ---
    void periodicTimeoutCheck();
    void publishTargetCommand(float pitchOffsetDeg, float angVelDps);
    void applyConfig(const ControlConfig& controlConf, const SystemBehaviorConfig& behaviorConf);
    esp_err_t startTimeoutTimer();
    esp_err_t stopTimeoutTimer();
    static void timeout_timer_callback(void* arg);
};
