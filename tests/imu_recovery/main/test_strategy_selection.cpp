#include "unity.h"

#include "BalancingAlgorithm.hpp"
#include "CONFIG_FullConfigUpdate.hpp"
#include "CONTROL_RunModeChanged.hpp"
#include "EventBus.hpp"

namespace {
ConfigData makeLongitudinalConfig()
{
    ConfigData config;
    config.control.strategies.nested_pid.angle = {
        15.0f, 0.0f, 0.0f, -720.0f, 720.0f, -10.0f, 10.0f
    };
    config.control.strategies.nested_pid.speed_left = {
        0.01f, 0.05f, 0.0001f, -1.0f, 1.0f, -10.0f, 10.0f
    };
    config.control.strategies.nested_pid.speed_right =
        config.control.strategies.nested_pid.speed_left;
    config.control.strategies.nested_pid.yaw_angle = {
        2.0f, 0.0f, 0.05f, -60.0f, 60.0f, -20.0f, 20.0f
    };
    config.control.strategies.nested_pid.yaw_rate = {
        0.1f, 0.01f, 0.005f, -100.0f, 100.0f, -50.0f, 50.0f
    };
    config.control.max_target_pitch_offset_deg =
        config.control.strategies.nested_pid.max_target_pitch_offset_deg;
    config.control.yaw_control_enabled =
        config.control.strategies.nested_pid.yaw_control_enabled;

    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.pitch = {
        0.2f, 0.0f, 0.01f, -1.0f, 1.0f, -1.0f, 1.0f
    };
    longitudinal.velocity = {
        0.1f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0f, 1.0f
    };
    longitudinal.max_pitch_offset_deg = 5.0f;
    longitudinal.max_pitch_rate_dps = 90.0f;
    longitudinal.max_effort = 0.8f;
    longitudinal.configured = true;
    longitudinal.revision = 1;
    return config;
}
}

TEST_CASE("strategy selection is applied only while control is disabled", "[control][strategy]")
{
    EventBus& bus = EventBus::getInstance();
    const ConfigData initial = makeLongitudinalConfig();
    BalancingAlgorithm algorithm(
        bus,
        initial.control.strategies.nested_pid.angle,
        initial.control.strategies.nested_pid.speed_left,
        initial.control.strategies.nested_pid.speed_right,
        initial.control.strategies.nested_pid.yaw_angle,
        initial.control.strategies.nested_pid.yaw_rate,
        initial.control,
        initial.encoder,
        initial.dimensions);

    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceStrategyId::NESTED_PID),
                          static_cast<int>(algorithm.getActiveStrategyId()));

    ConfigData longitudinal = initial;
    longitudinal.control.strategies.active = BalanceStrategyId::LONGITUDINAL_CASCADE;
    algorithm.handleEvent(CONFIG_FullConfigUpdate(longitudinal));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceStrategyId::LONGITUDINAL_CASCADE),
                          static_cast<int>(algorithm.getActiveStrategyId()));

    algorithm.handleEvent(CONTROL_RunModeChanged(
        ControlRunMode::BALANCING, 1, true, 1, 1));
    ConfigData nested = longitudinal;
    nested.control.strategies.active = BalanceStrategyId::NESTED_PID;
    algorithm.handleEvent(CONFIG_FullConfigUpdate(nested));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceStrategyId::LONGITUDINAL_CASCADE),
                          static_cast<int>(algorithm.getActiveStrategyId()));

    algorithm.handleEvent(CONTROL_RunModeChanged(
        ControlRunMode::DISABLED, 0, false, 2, 1));
    algorithm.handleEvent(CONFIG_FullConfigUpdate(nested));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BalanceStrategyId::NESTED_PID),
                          static_cast<int>(algorithm.getActiveStrategyId()));
}
