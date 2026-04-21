#pragma once

#include <mutex>
#include "esp_err.h"
#include "EventHandler.hpp"
#include <memory>

// Forward Declarations
class OrientationEstimator;
class BalancingAlgorithm;
class EncoderService;
class MotorService;
class BatteryService;
class StateManager;
class CommandProcessor;
class BaseEvent;
class MOTION_TargetMovement;
class EventBus;

class RobotController : public EventHandler {
public:
    RobotController(
        std::shared_ptr<OrientationEstimator> estimator,
        EncoderService& encoderService,
        MotorService& motorService,
        BalancingAlgorithm& algorithm,
        StateManager& stateManager,
        BatteryService& batteryService,
        CommandProcessor& commandProcessor
    );

    esp_err_t init(EventBus& bus);
    void runControlStep(float dt);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

private:
    static constexpr const char* TAG = "RobotController";

    // References to components
    std::shared_ptr<OrientationEstimator> m_estimator;
    EncoderService& m_encoderService;
    MotorService& m_motorService;
    BalancingAlgorithm& m_algorithm;
    StateManager& m_stateManager;
    BatteryService& m_batteryService;
    CommandProcessor& m_commandProcessor;

    // Latest command values (thread-safe)
    float m_latestTargetPitchOffset_deg = 0.0f;
    float m_latestTargetAngVel_dps = 0.0f;
    std::mutex m_target_values_mutex;

    // Event bus reference
    EventBus* m_eventBus = nullptr;

    void handleTargetMovementCommand(const MOTION_TargetMovement& event);
};