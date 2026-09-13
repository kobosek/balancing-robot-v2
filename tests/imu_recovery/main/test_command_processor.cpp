#include "unity.h"

#include "CommandProcessor.hpp"
#include "COMMAND_InputModeChanged.hpp"
#include "EventBus.hpp"
#include "EventHandler.hpp"
#include "MOTION_TargetLinearVelocity.hpp"
#include "UI_JoystickInput.hpp"

#include <atomic>
#include <memory>
#include <string>

namespace {

class LinearCommandObserver final : public EventHandler {
public:
    std::atomic<unsigned> count{0};
    std::atomic<uint64_t> lastSequence{0};
    std::atomic<uint64_t> lastArmId{0};
    std::atomic<bool> lastStop{false};
    std::atomic<float> lastVelocityMps{0.0f};

    void handleEvent(const BaseEvent& event) override
    {
        if (!event.is<MOTION_TargetLinearVelocity>()) return;
        const auto& command = event.as<MOTION_TargetLinearVelocity>();
        lastSequence.store(command.sequence, std::memory_order_relaxed);
        lastArmId.store(command.armId, std::memory_order_relaxed);
        lastStop.store(command.stop, std::memory_order_relaxed);
        lastVelocityMps.store(command.targetVelocityMps, std::memory_order_relaxed);
        count.fetch_add(1, std::memory_order_relaxed);
    }

    std::string getHandlerName() const override
    {
        return "CommandProcessorTestObserver";
    }
};

std::shared_ptr<LinearCommandObserver> commandObserver()
{
    static auto value = std::make_shared<LinearCommandObserver>();
    static const bool subscribed = [] {
        EventBus::getInstance().subscribe<MOTION_TargetLinearVelocity>(value);
        return true;
    }();
    (void)subscribed;
    value->count.store(0, std::memory_order_relaxed);
    value->lastSequence.store(0, std::memory_order_relaxed);
    value->lastArmId.store(0, std::memory_order_relaxed);
    value->lastStop.store(false, std::memory_order_relaxed);
    value->lastVelocityMps.store(0.0f, std::memory_order_relaxed);
    return value;
}

ControlConfig longitudinalControlConfig()
{
    ControlConfig config;
    config.strategies.active = BalanceStrategyId::LONGITUDINAL_CASCADE;
    config.strategies.longitudinal_cascade.max_velocity_mps = 0.4f;
    return config;
}

} // namespace

TEST_CASE("longitudinal joystick requires the current server session token",
          "[control][commands][session]")
{
    auto observed = commandObserver();
    CommandProcessor processor(EventBus::getInstance(),
                               longitudinalControlConfig(),
                               SystemBehaviorConfig{});
    TEST_ASSERT_EQUAL(ESP_OK, processor.init());

    processor.handleEvent(COMMAND_InputModeChanged(true, 7));
    TEST_ASSERT_EQUAL_UINT(1, observed->count.load(std::memory_order_relaxed));
    TEST_ASSERT_TRUE(observed->lastStop.load(std::memory_order_relaxed));
    TEST_ASSERT_EQUAL_UINT64(7, observed->lastArmId.load(std::memory_order_relaxed));

    // A legacy client that omits the token must not refresh timeout state or
    // publish a new longitudinal request in the active session.
    processor.handleEvent(UI_JoystickInput(0.0f, -1.0f, 0, 1));
    TEST_ASSERT_EQUAL_UINT(1, observed->count.load(std::memory_order_relaxed));

    processor.handleEvent(UI_JoystickInput(0.0f, -1.0f, 7, 1));
    TEST_ASSERT_EQUAL_UINT(2, observed->count.load(std::memory_order_relaxed));
    TEST_ASSERT_FALSE(observed->lastStop.load(std::memory_order_relaxed));
    TEST_ASSERT_EQUAL_UINT64(7, observed->lastArmId.load(std::memory_order_relaxed));
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.4f,
                             observed->lastVelocityMps.load(std::memory_order_relaxed));

    // Reordered or duplicated producer packets cannot refresh the command.
    processor.handleEvent(UI_JoystickInput(0.0f, -0.5f, 7, 1));
    processor.handleEvent(UI_JoystickInput(0.0f, -0.5f, 7, 0));
    TEST_ASSERT_EQUAL_UINT(2, observed->count.load(std::memory_order_relaxed));

    processor.handleEvent(UI_JoystickInput(0.0f, -0.5f, 7, 2));
    TEST_ASSERT_EQUAL_UINT(3, observed->count.load(std::memory_order_relaxed));

    processor.handleEvent(COMMAND_InputModeChanged(false, 7));
    TEST_ASSERT_TRUE(observed->lastStop.load(std::memory_order_relaxed));
}
