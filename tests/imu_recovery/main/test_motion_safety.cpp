#include "unity.h"
#include "MotorService.hpp"
#include "MOTOR_OutputEnabledChanged.hpp"
#include "StateManagerPolicy.hpp"
#include "fakes.hpp"
#include "freertos/task.h"
#include <limits>

TEST_CASE("IMU faults exit every motor-active mode", "[imu][policy]") {
    using namespace state_manager_policy;
    for (auto state : {SystemState::BALANCING, SystemState::PID_TUNING,
                       SystemState::GUIDED_CALIBRATION, SystemState::FALLEN}) {
        TEST_ASSERT_TRUE(shouldReturnToIdleOnImuError(state));
    }
    for (auto state : {SystemState::INIT, SystemState::IDLE,
                       SystemState::SHUTDOWN, SystemState::FATAL_ERROR}) {
        TEST_ASSERT_FALSE(shouldReturnToIdleOnImuError(state));
    }
}

TEST_CASE("IMU hardware changes require an inactive operational state", "[imu][policy]") {
    using namespace state_manager_policy;
    TEST_ASSERT_TRUE(isImuHardwareConfigApplyAllowed(SystemState::IDLE));
    TEST_ASSERT_TRUE(isImuHardwareConfigApplyAllowed(SystemState::FALLEN));
    for (auto state : {SystemState::INIT, SystemState::BALANCING,
                       SystemState::PID_TUNING, SystemState::GUIDED_CALIBRATION,
                       SystemState::SHUTDOWN, SystemState::FATAL_ERROR}) {
        TEST_ASSERT_FALSE(isImuHardwareConfigApplyAllowed(state));
    }
}

TEST_CASE("disabled and non-finite effort commits zero to both motors", "[imu][motor]") {
    motor_fake::reset();
    MotorService motor(MotorConfig{}, EventBus::getInstance());
    TEST_ASSERT_EQUAL(ESP_OK, motor.setMotorEffort(1.0f, -1.0f));
    motor.handleEvent(MOTOR_OutputEnabledChanged(true, 1, 1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        motor.setMotorEffort(std::numeric_limits<float>::quiet_NaN(), 1.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        motor.setMotorEffort(1.0f, std::numeric_limits<float>::infinity()));
    TEST_ASSERT_EQUAL_UINT(6, motor_fake::count.load());
    for (unsigned i = 0; i < 6; ++i) {
        TEST_ASSERT_EQUAL_UINT32(0, motor_fake::writes[i].duty1);
        TEST_ASSERT_EQUAL_UINT32(0, motor_fake::writes[i].duty2);
    }
}

namespace {
struct RaceContext {
    MotorService* motor;
    SemaphoreHandle_t writerDone;
    SemaphoreHandle_t stopStarted;
    SemaphoreHandle_t stopDone;
};
void writeTask(void* argument) {
    auto& context = *static_cast<RaceContext*>(argument);
    context.motor->setMotorEffort(0.5f, 0.5f, 1, 1);
    xSemaphoreGive(context.writerDone);
    vTaskDelete(nullptr);
}
void stopTask(void* argument) {
    auto& context = *static_cast<RaceContext*>(argument);
    xSemaphoreGive(context.stopStarted);
    context.motor->handleEvent(MOTOR_OutputEnabledChanged(false, 2, 1));
    xSemaphoreGive(context.stopDone);
    vTaskDelete(nullptr);
}
}

TEST_CASE("stop serializes with an in-flight two-motor commit", "[imu][motor]") {
    motor_fake::reset();
    MotorService motor(MotorConfig{}, EventBus::getInstance());
    motor.handleEvent(MOTOR_OutputEnabledChanged(true, 1, 1));
    motor_fake::writeEntered = xSemaphoreCreateBinary();
    motor_fake::releaseWrite = xSemaphoreCreateBinary();
    RaceContext context{&motor, xSemaphoreCreateBinary(), xSemaphoreCreateBinary(),
                        xSemaphoreCreateBinary()};
    configASSERT(motor_fake::writeEntered && motor_fake::releaseWrite &&
                 context.writerDone && context.stopStarted && context.stopDone);
    motor_fake::pauseNextWrite = true;
    configASSERT(xTaskCreate(writeTask, "test_write", 4096, &context, 5, nullptr) == pdPASS);
    xSemaphoreTake(motor_fake::writeEntered, portMAX_DELAY);
    configASSERT(xTaskCreate(stopTask, "test_stop", 4096, &context, 5, nullptr) == pdPASS);
    xSemaphoreTake(context.stopStarted, portMAX_DELAY);
    // Capture the result, but only assert after the tasks release local objects.
    const bool stopFinishedEarly = xSemaphoreTake(context.stopDone, pdMS_TO_TICKS(20)) == pdTRUE;
    xSemaphoreGive(motor_fake::releaseWrite);
    xSemaphoreTake(context.writerDone, portMAX_DELAY);
    if (!stopFinishedEarly) {
        xSemaphoreTake(context.stopDone, portMAX_DELAY);
    }
    motor.setMotorEffort(1.0f, 1.0f);
    vSemaphoreDelete(context.writerDone);
    vSemaphoreDelete(context.stopStarted);
    vSemaphoreDelete(context.stopDone);
    vSemaphoreDelete(motor_fake::writeEntered);
    vSemaphoreDelete(motor_fake::releaseWrite);
    motor_fake::writeEntered = motor_fake::releaseWrite = nullptr;
    TEST_ASSERT_FALSE(stopFinishedEarly);
    TEST_ASSERT_EQUAL_UINT(6, motor_fake::count.load());
    TEST_ASSERT_GREATER_THAN_UINT32(0, motor_fake::writes[0].duty1);
    TEST_ASSERT_GREATER_THAN_UINT32(0, motor_fake::writes[1].duty1);
    for (unsigned i = 2; i < 6; ++i) {
        TEST_ASSERT_EQUAL_UINT32(0, motor_fake::writes[i].duty1);
        TEST_ASSERT_EQUAL_UINT32(0, motor_fake::writes[i].duty2);
    }
}

#include "sensor_fakes.hpp"
#include "ControlEventDispatcher.hpp"
#include "CONTROL_ImuDataInvalid.hpp"
#include "BalanceMonitor.hpp"
#include "IMU_AvailabilityChanged.hpp"
#include "IMU_OrientationData.hpp"
#include "BALANCE_AutoBalanceReady.hpp"
#include "BALANCE_MonitorModeChanged.hpp"
#include "esp_timer.h"

TEST_CASE("expired commit revokes old arm even after data returns", "[imu][motor]") {
    motor_fake::reset(); sensor_fake::reset(); sensor_fake::clockUs = 1000000;
    MotorService motor(MotorConfig{}, EventBus::getInstance());
    motor.handleEvent(MOTOR_OutputEnabledChanged(true, 1, 4));
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, motor.setMotorEffort(1, 1, 1, 4, 979999, 20000));
    motor.handleEvent(MOTOR_OutputEnabledChanged(true, 1, 4));
    TEST_ASSERT_FALSE(motor.isArmAllowed(1, 4));
    motor.setMotorEffort(1, 1, 1, 4, 999000, 20000);
    for (unsigned i = 0; i < motor_fake::count; ++i) {
        TEST_ASSERT_EQUAL_UINT32(0, motor_fake::writes[i].duty1);
        TEST_ASSERT_EQUAL_UINT32(0, motor_fake::writes[i].duty2);
    }
    motor.handleEvent(MOTOR_OutputEnabledChanged(false, 2, 4));
    motor.handleEvent(MOTOR_OutputEnabledChanged(true, 3, 5));
    motor.inhibitImu(1); // Delayed old-arm fault cannot revoke a later decision.
    TEST_ASSERT_TRUE(motor.isArmAllowed(3, 5));
    TEST_ASSERT_FALSE(motor.isArmAllowed(3, 4));
    sensor_fake::clockUs = -1;
}

namespace {
class SafetyObserver : public EventHandler {
public:
    std::atomic<unsigned> faults{0}, autoReady{0};
    std::atomic<uint64_t> lastArm{0};
    void handleEvent(const BaseEvent& event) override {
        if (event.is<CONTROL_ImuDataInvalid>()) { lastArm = event.as<CONTROL_ImuDataInvalid>().fault.armId; ++faults; }
        if (event.is<BALANCE_AutoBalanceReady>()) ++autoReady;
    }
    std::string getHandlerName() const override { return "SafetyTestObserver"; }
};
std::shared_ptr<SafetyObserver> observer() {
    static auto value = std::make_shared<SafetyObserver>();
    static const bool subscribed = [] {
        EventBus::getInstance().subscribe<CONTROL_ImuDataInvalid, BALANCE_AutoBalanceReady>(value);
        return true;
    }();
    (void)subscribed;
    value->faults = 0; value->autoReady = 0; value->lastArm = 0;
    return value;
}
}
TEST_CASE("fault latch survives a saturated telemetry queue", "[imu][dispatch]") {
    auto observed = observer();
    ControlEventDispatcher dispatcher(EventBus::getInstance(), 1);
    TEST_ASSERT_EQUAL(ESP_OK, dispatcher.init());
    for (unsigned i = 0; i < 100; ++i) dispatcher.enqueueTelemetry({});
    dispatcher.latchImuFault({7, 4, esp_timer_get_time(), IMUFaultReason::STALE});
    dispatcher.latchImuFault({8, 4, esp_timer_get_time(), IMUFaultReason::STALE});
    TEST_ASSERT_TRUE(dispatcher.start(5));
    const auto deadline = esp_timer_get_time() + 200000;
    while (!observed->faults && esp_timer_get_time() < deadline) vTaskDelay(1);
    dispatcher.stop();
    TEST_ASSERT_GREATER_THAN_UINT(0, observed->faults.load());
    TEST_ASSERT_TRUE(observed->lastArm == 8);
    TEST_ASSERT_GREATER_THAN_UINT32(0, dispatcher.getDroppedEventCount());
}
TEST_CASE("auto balance needs a fresh complete hold and ignores duplicate frames", "[imu][monitor]") {
    auto observed = observer();
    sensor_fake::clockUs = 1000000;
    SystemBehaviorConfig config; config.auto_balance_hold_duration_ms = 40;
    BalanceMonitor monitor(EventBus::getInstance(), config);
    monitor.handleEvent(IMU_AvailabilityChanged(true, 3, 1));
    monitor.handleEvent(BALANCE_MonitorModeChanged(false, true));
    OrientationEstimate sample; sample.valid = true; sample.generation = 3;
    sample.sample_timestamp_us = 1004000; sample.sample_sequence = 1;
    sensor_fake::clockUs = 1004000;
    monitor.handleEvent(IMU_OrientationData(sample));
    for (unsigned i = 0; i < 20; ++i) monitor.handleEvent(IMU_OrientationData(sample));
    TEST_ASSERT_EQUAL_UINT(0, observed->autoReady.load());
    // A gap exceeds the age budget, restarting the entire hold.
    sensor_fake::clockUs = 1060000; sample.sample_timestamp_us = 1060000; ++sample.sample_sequence;
    monitor.handleEvent(IMU_OrientationData(sample));
    for (unsigned i = 0; i < 9; ++i) {
        sensor_fake::clockUs += 4000; sample.sample_timestamp_us += 4000; ++sample.sample_sequence;
        monitor.handleEvent(IMU_OrientationData(sample));
    }
    TEST_ASSERT_EQUAL_UINT(0, observed->autoReady.load());
    sensor_fake::clockUs += 4000; sample.sample_timestamp_us += 4000; ++sample.sample_sequence;
    monitor.handleEvent(IMU_OrientationData(sample));
    TEST_ASSERT_EQUAL_UINT(1, observed->autoReady.load());
    monitor.handleEvent(IMU_AvailabilityChanged(false, 4, 2));
    monitor.handleEvent(IMU_AvailabilityChanged(true, 3, 1)); // superseded
    sample.sample_timestamp_us += 4000; ++sample.sample_sequence; sensor_fake::clockUs += 4000;
    monitor.handleEvent(IMU_OrientationData(sample));
    TEST_ASSERT_EQUAL_UINT(1, observed->autoReady.load());
    sensor_fake::clockUs = -1;
}
