// main/algorithms/include/CommandProcessor.hpp
#pragma once
#include "EventBus.hpp"
#include "SystemState.hpp"
#include "SystemStateChangedEvent.hpp"
#include "TargetMovementCommand.hpp" // Output event
#include "JoystickInputEvent.hpp"    // <<<--- Input event
#include "esp_log.h"
#include <mutex>                     // <<<--- ADDED For thread safety

class CommandProcessor {
public:
    CommandProcessor(EventBus& bus);
    ~CommandProcessor() = default;

    esp_err_t init(); // Subscribe to events

    // Getters for RobotController (Thread-safe)
    float getTargetLinearVelocity() const;
    float getTargetAngularVelocity() const;

private:
    static constexpr const char* TAG = "CommandProc";
    // --- Configuration / Tuning ---
    static constexpr float JOYSTICK_DEADZONE = 0.10f; // Ignore small inputs
    static constexpr uint64_t INPUT_TIMEOUT_US = 500 * 1000; // 500ms timeout
    // Max speeds (Load from config eventually?)
    float m_max_linear_velocity = 0.3f;     // Max m/s from joystick Y
    float m_max_angular_velocity_dps = 60.0f; // Max dps from joystick X

    EventBus& m_eventBus;
    SystemState m_current_state = SystemState::INIT;

    // --- Target State (Protected) ---
    float m_target_linear_velocity = 0.0f;    // m/s
    float m_target_angular_velocity = 0.0f;   // dps
    mutable std::mutex m_target_mutex;        // <<<--- ADDED Mutex

    // --- Timeout Tracking ---
    int64_t m_last_input_time_us = 0;
    bool m_input_timed_out = true;

    // Event Handlers
    void handleSystemStateChange(const SystemStateChangedEvent& event);
    void handleJoystickInput(const JoystickInputEvent& event); // <<<--- Changed Handler

    // Internal Helpers
    void checkInputTimeout();
    void publishTargetCommand(float linVel, float angVelDps);
};