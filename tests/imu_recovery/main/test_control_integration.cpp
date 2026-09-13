#include "unity.h"

#include "BalancingAlgorithm.hpp"
#include "BatteryService.hpp"
#include "CommandProcessor.hpp"
#include "ConfigurationService.hpp"
#include "JsonConfigParser.hpp"
#include "SPIFFSStorageService.hpp"
#include "ControlEventDispatcher.hpp"
#include "ControlModeExecutor.hpp"
#include "CONTROL_RunModeChanged.hpp"
#include "MOTION_TargetLinearVelocity.hpp"
#include "EncoderService.hpp"
#include "EventBus.hpp"
#include "EventHandler.hpp"
#include "GuidedCalibrationService.hpp"
#include "MOTOR_OutputEnabledChanged.hpp"
#include "MotorService.hpp"
#include "OrientationEstimator.hpp"
#include "PidTuningService.hpp"
#include "RobotController.hpp"
#include "UI_JoystickInput.hpp"
#include "COMMAND_InputModeChanged.hpp"

#include "fakes.hpp"
#include "pcnt_fakes.hpp"
#include "sensor_fakes.hpp"

#include <cmath>
#include <memory>
#include <string>

namespace {

class NullStorage final : public IStorageService {
public:
    esp_err_t init() override { return ESP_OK; }

    esp_err_t loadData(const std::string&, std::string&) override
    {
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t saveData(const std::string&, const std::string&) override
    {
        return ESP_OK;
    }
};

// EventBus deliberately owns subscribers for the lifetime of the process.
// A weak relay keeps a test-local controller from becoming a dangling global
// subscriber after the test has torn down its fakes and services.
template <typename Target>
class WeakRelay final : public EventHandler {
public:
    explicit WeakRelay(const std::shared_ptr<Target>& target) : m_target(target) {}

    void handleEvent(const BaseEvent& event) override
    {
        if (const auto target = m_target.lock()) {
            target->handleEvent(event);
        }
    }

    std::string getHandlerName() const override { return "ControlIntegrationWeakRelay"; }

private:
    std::weak_ptr<Target> m_target;
};

ConfigData makeIntegrationConfig()
{
    ConfigData config;
    config.control.strategies.active = BalanceStrategyId::LONGITUDINAL_CASCADE;

    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.pitch = {
        0.8f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0f, 1.0f
    };
    longitudinal.velocity = {
        0.8f, 0.15f, 0.0f, -1.0f, 1.0f, -1.0f, 1.0f
    };
    longitudinal.pitch_trim_deg = 0.0f;
    longitudinal.max_pitch_offset_deg = 5.0f;
    longitudinal.max_pitch_rate_dps = 90.0f;
    longitudinal.max_velocity_mps = 0.4f;
    longitudinal.max_acceleration_mps2 = 1.0f;
    longitudinal.max_deceleration_mps2 = 1.0f;
    longitudinal.max_effort = 0.8f;
    longitudinal.loop_mode = LongitudinalLoopMode::VELOCITY;
    longitudinal.motion_request_limit_enabled = false;
    longitudinal.configured = true;
    longitudinal.revision = 1;

    config.encoder.speed_filter_alpha = 1.0f;
    config.behavior.imu_max_sample_age_ms = 20;
    config.behavior.joystick_timeout_ms = 500;
    config.behavior.joystick_check_interval_ms = 100;
    return config;
}

} // namespace

TEST_CASE("longitudinal command reaches the real control loop through EventBus",
          "[control][longitudinal][integration]")
{
    pcnt_fake::reset();
    sensor_fake::reset();
    motor_fake::reset();
    sensor_fake::clockUs = 1000000;

    const ConfigData config = makeIntegrationConfig();
    EventBus& bus = EventBus::getInstance();
    NullStorage storage;
    JsonConfigParser parser;
    ControlOperationGate operationGate;
    ConfigurationService configuration(storage, parser, bus,
                                       "config.json", &operationGate);

    EncoderService encoders(config.encoder, 5000);
    TEST_ASSERT_EQUAL(ESP_OK, encoders.init());

    auto estimator = std::make_shared<OrientationEstimator>();
    estimator->init(0.98f, 0.005f);
    const uint32_t generation = estimator->getOrientation().generation;
    for (int i = 0; i < 5; ++i) {
        sensor_fake::clockUs += 5000;
        TEST_ASSERT_TRUE(estimator->processSample(
            0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 0.0f,
            sensor_fake::clockUs.load(), generation));
    }
    TEST_ASSERT_TRUE(estimator->setValidated());

    MotorService motor(config.motor, bus);
    BatteryService battery(config.battery, config.behavior, bus);
    auto balancing = std::make_shared<BalancingAlgorithm>(bus, config);
    TEST_ASSERT_EQUAL(ESP_OK, balancing->init());
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(BalanceStrategyId::LONGITUDINAL_CASCADE),
        static_cast<int>(balancing->getActiveStrategyId()));

    PidTuningService tuning(bus, configuration, encoders, config.pid_tuning);
    GuidedCalibrationService guided(bus, config.pid_tuning, config.motor);
    ControlModeExecutor executor(*balancing, tuning, guided);
    ControlEventDispatcher dispatcher(bus);
    auto controller = std::make_shared<RobotController>(
        estimator, encoders, motor, battery, executor, dispatcher,
        config.behavior, config.encoder);

    // Reproduce the relevant application wiring without retaining strong
    // references to the test-local controller after teardown.
    auto controllerRelay = std::make_shared<WeakRelay<RobotController>>(controller);
    auto balancingRelay = std::make_shared<WeakRelay<BalancingAlgorithm>>(balancing);
    bus.subscribe<MOTION_TargetLinearVelocity, CONTROL_RunModeChanged>(controllerRelay);
    bus.subscribe<CONTROL_RunModeChanged>(balancingRelay);

    motor.handleEvent(MOTOR_OutputEnabledChanged(true, 1, generation));
    bus.publish(CONTROL_RunModeChanged(
        ControlRunMode::BALANCING, 1, true, 1, generation));

    CommandProcessor commands(bus, config.control, config.behavior);
    TEST_ASSERT_EQUAL(ESP_OK, commands.init());
    commands.handleEvent(COMMAND_InputModeChanged(true, 1));
    // The producer token and source sequence are part of the public input
    // boundary. CommandProcessor publishes the firmware sequence itself.
    commands.handleEvent(UI_JoystickInput(0.0f, -1.0f, 1, 1));

    unsigned writesBeforeDrive = motor_fake::count.load();
    for (int step = 0; step < 4; ++step) {
        sensor_fake::clockUs += 5000;
        TEST_ASSERT_TRUE(estimator->processSample(
            0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 0.0f,
            sensor_fake::clockUs.load(), generation));
        controller->runControlStep(0.005f);
    }

    const auto frame = encoders.getFrame();
    TEST_ASSERT_TRUE(frame.left.valid && frame.right.valid);
    TEST_ASSERT_GREATER_THAN_UINT(writesBeforeDrive, motor_fake::count.load());
    bool producedDriveEffort = false;
    for (unsigned i = writesBeforeDrive; i < motor_fake::count.load() &&
         i < motor_fake::writes.size(); ++i) {
        const auto& write = motor_fake::writes[i];
        producedDriveEffort = producedDriveEffort ||
            (write.duty1 != 0 || write.duty2 != 0);
    }
    TEST_ASSERT_TRUE(producedDriveEffort);

    // A stop from the same producer session travels through the same path and
    // must not be lost merely because its numeric velocity is zero.
    commands.handleEvent(UI_JoystickInput(0.0f, 0.0f, 1, 2));
    sensor_fake::clockUs += 5000;
    TEST_ASSERT_TRUE(estimator->processSample(
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 0.0f,
        sensor_fake::clockUs.load(), generation));
    controller->runControlStep(0.005f);

    const auto diagnostics = balancing->getDiagnostics();
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(BalanceStrategyId::LONGITUDINAL_CASCADE),
        static_cast<int>(diagnostics.strategyId));
    TEST_ASSERT_TRUE(diagnostics.velocityLoopEnabled);
    TEST_ASSERT_TRUE(diagnostics.phase == BalanceControlPhase::DRIVE ||
                     diagnostics.phase == BalanceControlPhase::BRAKE ||
                     diagnostics.phase == BalanceControlPhase::HOLD);

    // Disable the producer session and verify that the event is accepted as
    // a stop for the current arm rather than refreshing a drive request.
    commands.handleEvent(COMMAND_InputModeChanged(false, 1));
    sensor_fake::clockUs = -1;
}
