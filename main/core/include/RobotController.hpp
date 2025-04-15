// ================================================
// File: main/core/include/RobotController.hpp
// ================================================
#pragma once

#include <mutex>
#include "esp_err.h"

// Forward Declarations
class OrientationEstimator;
class BalancingAlgorithm;
class FallDetector;
class EncoderService;
class MotorService;
class BatteryService;
class StateManager;
class WebServer;
class CommandProcessor;
class BaseEvent;
class TargetMovementCommand;
class EventBus;

class RobotController {
public:
    RobotController(
        OrientationEstimator& estimator,
        EncoderService& encoderService,
        MotorService& motorService,
        BalancingAlgorithm& algorithm,
        StateManager& stateManager,
        FallDetector& fallDetector,
        WebServer& webServer,
        BatteryService& batteryService,
        CommandProcessor& commandProcessor
    );

    esp_err_t init(EventBus& bus);
    void runControlStep(float dt);

private:
    static constexpr const char* TAG = "RobotController";

    // References to components
    OrientationEstimator& m_estimator;
    EncoderService& m_encoderService;
    MotorService& m_motorService;
    BalancingAlgorithm& m_algorithm;
    StateManager& m_stateManager;
    FallDetector& m_fallDetector;
    WebServer& m_webServer;
    BatteryService& m_batteryService;
    CommandProcessor& m_commandProcessor;

    // Latest command values (thread-safe)
    float m_latestTargetPitchOffset_deg = 0.0f;
    float m_latestTargetAngVel_dps = 0.0f;
    std::mutex m_target_values_mutex;

    // Event bus reference
    EventBus* m_eventBus = nullptr;

    void handleTargetMovementCommand(const BaseEvent& event);
};