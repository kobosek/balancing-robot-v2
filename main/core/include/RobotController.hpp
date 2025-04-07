// main/core/include/RobotController.hpp
#pragma once

// Forward Declarations
class OrientationEstimator;
class BalancingAlgorithm;
class FallDetector;
class EncoderService;
class MotorService;
class BatteryService;
class StateManager;
class WebServer;
class CommandProcessor;     // <<<--- Already forward declared

class RobotController {
public:
    // Constructor takes references to all required components
    RobotController(
        OrientationEstimator& estimator,
        EncoderService& encoderService,
        MotorService& motorService,
        BalancingAlgorithm& algorithm,
        StateManager& stateManager,
        FallDetector& fallDetector,
        WebServer& webServer,
        BatteryService& batteryService,
        CommandProcessor& commandProcessor // <<<--- Already added previously
    );

    // The main method called by the control task loop
    void runControlStep(float dt);

private:
    static constexpr const char* TAG = "RobotController";

    // Store references to the components
    OrientationEstimator& m_estimator;
    EncoderService& m_encoderService;
    MotorService& m_motorService;
    BalancingAlgorithm& m_algorithm;
    StateManager& m_stateManager;
    FallDetector& m_fallDetector;
    WebServer& m_webServer;
    BatteryService& m_batteryService;
    CommandProcessor& m_commandProcessor; // <<<--- Already added previously
};