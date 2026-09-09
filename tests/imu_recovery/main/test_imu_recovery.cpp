#include "unity.h"
#include "IMUService.hpp"
#include "OrientationEstimator.hpp"
#include "EventBus.hpp"
#include "IMU_SystemPolicyChanged.hpp"
#include "CONFIG_ImuConfigUpdate.hpp"
#include "IMU_CalibrationRequest.hpp"
#include "sensor_fakes.hpp"
#include "freertos/task.h"
#include "esp_timer.h"
#include <functional>
namespace {
bool waitFor(const std::function<bool()>& condition, unsigned timeoutMs = 1000) {
    const auto deadline = esp_timer_get_time() + timeoutMs * 1000LL;
    while (esp_timer_get_time() < deadline) {
        if (condition()) return true;
        vTaskDelay(pdMS_TO_TICKS(5) ? pdMS_TO_TICKS(5) : 1);
    }
    return condition();
}
}
TEST_CASE("worker validates real samples with IRQ fallback and joins on shutdown", "[imu][worker]") {
    sensor_fake::reset();
    auto estimator = std::make_shared<OrientationEstimator>();
    MPU6050Config config; config.int_pin = -1;
    IMUService service(estimator, config, SystemBehaviorConfig{}, EventBus::getInstance());
    TEST_ASSERT_EQUAL(ESP_OK, service.init());
    TEST_ASSERT_EQUAL_UINT(0, sensor_fake::opens);
    service.handleEvent(IMU_SystemPolicyChanged(true, true, true));
    TEST_ASSERT_TRUE(service.startTasks());
    const bool ready = waitFor([&] { return service.isAvailable(); });
    const auto status = service.getStatusSnapshot();
    uint32_t generation = 0;
    const bool armed = service.reserveMotion(generation);
    config.accel_range = 2;
    service.handleEvent(CONFIG_ImuConfigUpdate(config, true));
    vTaskDelay(pdMS_TO_TICKS(30));
    const bool pending = service.getStatusSnapshot().configurationPending;
    const auto oldGeneration = estimator->getOrientation().generation;
    service.releaseMotion();
    const bool applied = waitFor([&] { return service.isAvailable() && !service.getStatusSnapshot().configurationPending; });
    service.stopTasks();
    TEST_ASSERT_TRUE(ready);
    TEST_ASSERT_TRUE(armed);
    TEST_ASSERT_TRUE(status.ready && estimator->getOrientation().sample_sequence >= 5);
    TEST_ASSERT_TRUE(pending);
    TEST_ASSERT_EQUAL_UINT(generation, oldGeneration);
    TEST_ASSERT_TRUE(applied);
    TEST_ASSERT_FALSE(estimator->getOrientation().valid);
    TEST_ASSERT_EQUAL_UINT(0, sensor_fake::wrongOwner);
    TEST_ASSERT_EQUAL_UINT(sensor_fake::opens.load(), sensor_fake::closes.load());
}
TEST_CASE("absent sensor has bounded reconnect attempts and never arms", "[imu][worker]") {
    sensor_fake::reset(); sensor_fake::absent = true;
    auto estimator = std::make_shared<OrientationEstimator>();
    SystemBehaviorConfig behavior; behavior.imu_reconnect_interval_ms = 250;
    IMUService service(estimator, MPU6050Config{}, behavior, EventBus::getInstance());
    service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
    vTaskDelay(pdMS_TO_TICKS(700));
    uint32_t generation = 0;
    const bool armed = service.reserveMotion(generation);
    const auto status = service.getStatusSnapshot();
    service.stopTasks();
    TEST_ASSERT_FALSE(armed);
    TEST_ASSERT_FALSE(status.ready);
    TEST_ASSERT_TRUE(status.reconnectAttempts >= 2 && status.reconnectAttempts <= 3);
    TEST_ASSERT_EQUAL_UINT(0, sensor_fake::wrongOwner);
}
TEST_CASE("configured sensor without samples times out validation", "[imu][worker]") {
    sensor_fake::reset(); sensor_fake::noSamples = true;
    auto estimator = std::make_shared<OrientationEstimator>();
    IMUService service(estimator, MPU6050Config{}, SystemBehaviorConfig{}, EventBus::getInstance());
    service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
    const bool unavailable = waitFor([&] { return service.getCurrentState() == IMUState::UNAVAILABLE; });
    const auto status = service.getStatusSnapshot();
    service.stopTasks();
    TEST_ASSERT_TRUE(unavailable);
    TEST_ASSERT_FALSE(status.ready);
    TEST_ASSERT_EQUAL(static_cast<int>(IMUFaultReason::VALIDATION), static_cast<int>(status.lastReason));
}

#include "UI_Stop.hpp"
#include "StateManager.hpp"
#include "SystemState.hpp"
#include "UI_StartBalancing.hpp"
#include "UI_StartPidTuning.hpp"
#include "UI_StartGuidedCalibration.hpp"
#include "IMU_AvailabilityChanged.hpp"

TEST_CASE("failed FIFO repair escalates once to full reconnect", "[imu][worker]") {
    sensor_fake::reset();
    auto estimator = std::make_shared<OrientationEstimator>();
    MPU6050Config config; config.int_pin = -1;
    SystemBehaviorConfig behavior; behavior.imu_reconnect_interval_ms = 250;
    IMUService service(estimator, config, behavior, EventBus::getInstance());
    service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
    const bool ready = waitFor([&] { return service.isAvailable(); });
    const auto generation = estimator->getOrientation().generation;
    sensor_fake::failWriteRegister = 0x23; sensor_fake::writeFailures = 1;
    sensor_fake::fifoFailureConsume = 7;
    const bool failed = waitFor([&] { return service.getCurrentState() == IMUState::UNAVAILABLE; });
    const auto incident = service.getStatusSnapshot();
    const bool recovered = waitFor([&] { return service.isAvailable(); });
    const auto recoveredStatus = service.getStatusSnapshot();
    service.stopTasks();
    TEST_ASSERT_TRUE(ready && failed && recovered);
    TEST_ASSERT_FALSE(incident.ready);
    TEST_ASSERT_EQUAL_UINT32(1, incident.fifoResyncs);
    TEST_ASSERT_EQUAL_UINT32(2, recoveredStatus.reconnectAttempts);
    TEST_ASSERT_TRUE(recoveredStatus.generation > generation);
    TEST_ASSERT_EQUAL_UINT(0, sensor_fake::wrongOwner);
}

TEST_CASE("calibration cancel and disconnect invalidate the stream and exclude motion", "[imu][worker]") {
    for (const bool disconnect : {false, true}) {
        sensor_fake::reset();
        auto estimator = std::make_shared<OrientationEstimator>();
        MPU6050Config config; config.int_pin = -1; config.calibration_samples = 500;
        config.gyro_offset_x = 1.25f;
        IMUService service(estimator, config, SystemBehaviorConfig{}, EventBus::getInstance());
        service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
        const bool ready = waitFor([&] { return service.isAvailable(); });
        service.handleEvent(IMU_CalibrationRequest());
        const bool calibrating = waitFor([&] { return service.getCurrentState() == IMUState::CALIBRATION; });
        uint32_t generation = 0;
        const bool armed = service.reserveMotion(generation);
        const bool ota = service.reserveOta();
        if (disconnect) sensor_fake::absent = true;
        else service.handleEvent(UI_Stop());
        const bool ended = waitFor([&] { return service.getCurrentState() == IMUState::UNAVAILABLE; }, 300);
        const auto status = service.getStatusSnapshot();
        service.stopTasks();
        TEST_ASSERT_TRUE(ready && calibrating && ended);
        TEST_ASSERT_FALSE(armed || ota || status.ready);
        TEST_ASSERT_EQUAL(static_cast<int>(IMUFaultReason::CALIBRATION), static_cast<int>(status.lastReason));
        TEST_ASSERT_EQUAL_UINT(0, sensor_fake::wrongOwner);
    }
}

TEST_CASE("state manager rejects latent starts and exits all active modes on invalidity", "[imu][state]") {
    sensor_fake::reset();
    auto estimator = std::make_shared<OrientationEstimator>();
    MPU6050Config config; config.int_pin = -1;
    IMUService service(estimator, config, SystemBehaviorConfig{}, EventBus::getInstance());
    service.init();
    StateManager state(EventBus::getInstance(), SystemBehaviorConfig{}, BatteryConfig{});
    state.bindImu(service); state.markReady();
    state.handleEvent(UI_StartBalancing());
    const bool rejected = state.getStatusSnapshot().stateId == static_cast<int>(SystemState::IDLE);
    service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
    const bool ready = waitFor([&] { return service.isAvailable(); });
    auto status = service.getStatusSnapshot();
    state.handleEvent(IMU_AvailabilityChanged(true, status.generation, status.revision));
    const bool noLatentStart = state.getStatusSnapshot().stateId == static_cast<int>(SystemState::IDLE);
    bool modesStarted = true, modesStopped = true;
    for (unsigned mode = 0; mode < 3; ++mode) {
        if (mode == 0) state.handleEvent(UI_StartBalancing());
        if (mode == 1) state.handleEvent(UI_StartPidTuning(PidTuningTarget::MOTOR_SPEED_LEFT));
        if (mode == 2) state.handleEvent(UI_StartGuidedCalibration());
        const SystemState expected[] = {SystemState::BALANCING, SystemState::PID_TUNING, SystemState::GUIDED_CALIBRATION};
        modesStarted &= state.getStatusSnapshot().stateId == static_cast<int>(expected[mode]);
        state.handleEvent(IMU_AvailabilityChanged(false, status.generation, ++status.revision));
        modesStopped &= state.getStatusSnapshot().stateId == static_cast<int>(SystemState::IDLE);
        state.handleEvent(IMU_AvailabilityChanged(true, status.generation, ++status.revision));
    }
    state.handleEvent(UI_Stop());
    service.stopTasks();
    TEST_ASSERT_TRUE(ready && rejected && noLatentStart && modesStarted && modesStopped);
}

#include "I2CDevice.hpp"
#include "mpu6050.hpp"
#include "IMUCalibration.hpp"
TEST_CASE("failed or canceled gyro calibration never replaces saved offsets", "[imu][calibration]") {
    for (const bool cancel : {false, true}) {
        sensor_fake::reset();
        I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_1, GPIO_NUM_2, 0x68, 100000);
        MPU6050Driver driver(device); IMUCalibration calibration(driver);
        calibration.setOffsets(1.25f, -2.5f, 3.75f);
        if (!cancel) sensor_fake::absent = true;
        const auto result = calibration.calibrate(MPU6050Profile::fromConfig(MPU6050Config{}),
            20, nullptr, [cancel] { return cancel; });
        TEST_ASSERT_NOT_EQUAL(ESP_OK, result);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.25f, calibration.getGyroOffsetXDPS());
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, -2.5f, calibration.getGyroOffsetYDPS());
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.75f, calibration.getGyroOffsetZDPS());
    }
}

TEST_CASE("slow GPIO setup cannot fill FIFO before validation starts", "[imu][regression]") {
    sensor_fake::reset(); sensor_fake::irqSetupDelayMs = 50;
    auto estimator = std::make_shared<OrientationEstimator>();
    IMUService service(estimator, MPU6050Config{}, SystemBehaviorConfig{}, EventBus::getInstance());
    service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
    const bool ready = waitFor([&] { return service.isAvailable(); });
    const auto generation = estimator->getOrientation().generation;
    vTaskDelay(pdMS_TO_TICKS(300));
    const auto status = service.getStatusSnapshot();
    service.stopTasks();
    TEST_ASSERT_TRUE(ready && status.ready);
    TEST_ASSERT_EQUAL_UINT32(1, status.reconnectAttempts);
    TEST_ASSERT_EQUAL_UINT32(0, status.fifoResyncs);
    TEST_ASSERT_EQUAL_UINT32(generation, status.generation);
}

TEST_CASE("brief acquisition gap resumes without resetting a healthy sensor", "[imu][regression]") {
    sensor_fake::reset();
    auto estimator = std::make_shared<OrientationEstimator>();
    MPU6050Config config; config.int_pin = -1;
    IMUService service(estimator, config, SystemBehaviorConfig{}, EventBus::getInstance());
    service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
    const bool initialReady = waitFor([&] { return service.isAvailable(); });
    const auto before = service.getStatusSnapshot();
    sensor_fake::noSamples = true;
    vTaskDelay(pdMS_TO_TICKS(80));
    uint32_t generation = 0;
    const bool staleArmAccepted = service.reserveMotion(generation);
    const auto during = service.getStatusSnapshot();
    sensor_fake::noSamples = false;
    const bool recovered = waitFor([&] { return service.isAvailable(); });
    const auto after = service.getStatusSnapshot();
    service.stopTasks();
    TEST_ASSERT_TRUE(initialReady && recovered);
    TEST_ASSERT_FALSE(staleArmAccepted || during.ready);
    TEST_ASSERT_EQUAL_UINT32(before.generation, after.generation);
    TEST_ASSERT_EQUAL_UINT32(before.reconnectAttempts, after.reconnectAttempts);
    TEST_ASSERT_EQUAL_UINT32(0, after.fifoResyncs);
}
TEST_CASE("sustained absence of acquisition progress still reconnects", "[imu][regression]") {
    sensor_fake::reset();
    auto estimator = std::make_shared<OrientationEstimator>();
    IMUService service(estimator, MPU6050Config{}, SystemBehaviorConfig{}, EventBus::getInstance());
    service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
    const bool ready = waitFor([&] { return service.isAvailable(); });
    sensor_fake::noSamples = true;
    const bool unavailable = waitFor([&] { return service.getCurrentState() == IMUState::UNAVAILABLE; }, 500);
    service.stopTasks();
    TEST_ASSERT_TRUE(ready && unavailable);
}

TEST_CASE("gravity quality gates startup without reconnecting a healthy transport", "[imu][quality]") {
    sensor_fake::reset(); sensor_fake::fifoPose = 3;
    auto estimator = std::make_shared<OrientationEstimator>();
    MPU6050Config config; config.int_pin = -1;
    IMUService service(estimator, config, SystemBehaviorConfig{}, EventBus::getInstance());
    service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
    const bool receiving = waitFor([&] { return estimator->getOrientation().sample_sequence >= 5; });
    vTaskDelay(pdMS_TO_TICKS(350));
    const auto before = service.getStatusSnapshot();
    uint32_t generation = 0;
    const bool armed = service.reserveMotion(generation);
    sensor_fake::fifoPose = 0;
    const bool recovered = waitFor([&] { return service.isAvailable(); });
    const auto after = service.getStatusSnapshot();
    service.stopTasks();
    TEST_ASSERT_TRUE(receiving && recovered);
    TEST_ASSERT_FALSE(before.ready || armed);
    TEST_ASSERT_EQUAL_UINT32(1, after.reconnectAttempts);
    TEST_ASSERT_EQUAL_UINT32(before.generation, after.generation);
    TEST_ASSERT_EQUAL_UINT32(0, after.fifoResyncs);
}

TEST_CASE("legacy startup accepts unknown mounting and ordinary gravity scale error", "[imu][quality]") {
    for (const int pose : {1, 2, 4}) {
        sensor_fake::reset(); sensor_fake::fifoPose = pose;
        auto estimator = std::make_shared<OrientationEstimator>();
        MPU6050Config config; config.int_pin = -1;
        IMUService service(estimator, config, SystemBehaviorConfig{}, EventBus::getInstance());
        service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
        const bool ready = waitFor([&] { return service.isAvailable(); });
        uint32_t generation = 0;
        const bool armed = service.reserveMotion(generation);
        service.stopTasks();
        TEST_ASSERT_TRUE(ready);
        TEST_ASSERT_TRUE(armed);
    }
}

TEST_CASE("gyro clipping revokes generation without resetting FIFO hardware", "[imu][quality]") {
    sensor_fake::reset();
    auto estimator = std::make_shared<OrientationEstimator>();
    MPU6050Config config; config.int_pin = -1;
    IMUService service(estimator, config, SystemBehaviorConfig{}, EventBus::getInstance());
    service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
    const bool ready = waitFor([&] { return service.isAvailable(); });
    uint32_t generation = 0;
    const bool armed = service.reserveMotion(generation);
    sensor_fake::fifoSaturateAxes = 0x20;
    const bool revoked = waitFor([&] { return estimator->getOrientation().generation != generation; });
    sensor_fake::fifoSaturateAxes = 0;
    const bool recovered = waitFor([&] { return service.isAvailable(); });
    const auto status = service.getStatusSnapshot();
    service.releaseMotion(); service.stopTasks();
    TEST_ASSERT_TRUE(ready && armed && revoked && recovered);
    TEST_ASSERT_EQUAL_UINT32(1, status.reconnectAttempts);
    TEST_ASSERT_EQUAL_UINT32(0, status.fifoResyncs);
}

TEST_CASE("calibration rejects rotation and vibration and preserves saved offsets", "[imu][calibration]") {
    for (unsigned scenario = 0; scenario < 4; ++scenario) {
        sensor_fake::reset();
        I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_1, GPIO_NUM_2, 0x68, 100000);
        MPU6050Driver driver(device); IMUCalibration calibration(driver);
        calibration.setOffsets(0.1f, -0.1f, 0.2f);
        const auto profile = MPU6050Profile::fromConfig(MPU6050Config{});
        if (scenario == 0) sensor_fake::calibrationGyroZ = static_cast<int>(10 * profile.gyroLsbPerDps);
        if (scenario == 1) {
            sensor_fake::calibrationGyroZ = static_cast<int>(profile.gyroLsbPerDps);
            sensor_fake::calibrationAlternating = true;
        }
        if (scenario == 2) sensor_fake::calibrationAccelX = static_cast<int>(profile.accelLsbPerG);
        if (scenario == 3) sensor_fake::calibrationGyroZ = INT16_MAX;
        const auto result = calibration.calibrate(profile, 20, nullptr, {});
        TEST_ASSERT_NOT_EQUAL(ESP_OK, result);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1f, calibration.getGyroOffsetXDPS());
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.1f, calibration.getGyroOffsetYDPS());
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.2f, calibration.getGyroOffsetZDPS());
    }
}

TEST_CASE("stationary calibration accepts coherent motion samples", "[imu][calibration]") {
    sensor_fake::reset();
    I2CDevice device; device.open(I2C_NUM_0, GPIO_NUM_1, GPIO_NUM_2, 0x68, 100000);
    MPU6050Driver driver(device); IMUCalibration calibration(driver);
    const auto profile = MPU6050Profile::fromConfig(MPU6050Config{});
    sensor_fake::calibrationGyroZ = static_cast<int>(profile.gyroLsbPerDps);
    TEST_ASSERT_EQUAL(ESP_OK, calibration.calibrate(profile, 20, nullptr, {}));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, calibration.getGyroOffsetZDPS());
}

TEST_CASE("software configuration preserves hardware and only estimator changes revoke generation", "[imu][worker]") {
    sensor_fake::reset();
    auto estimator = std::make_shared<OrientationEstimator>();
    MPU6050Config config; config.int_pin = -1;
    IMUService service(estimator, config, SystemBehaviorConfig{}, EventBus::getInstance());
    service.init(); service.handleEvent(IMU_SystemPolicyChanged(true, true, true)); service.startTasks();
    const bool ready = waitFor([&] { return service.isAvailable(); });
    const auto before = service.getStatusSnapshot();
    config.calibration_samples += 1;
    service.handleEvent(CONFIG_ImuConfigUpdate(config, true));
    const bool applied = waitFor([&] { return !service.getStatusSnapshot().configurationPending; });
    const auto unchanged = service.getStatusSnapshot();
    config.comp_filter_alpha = 0.97f;
    service.handleEvent(CONFIG_ImuConfigUpdate(config, true));
    const bool revalidated = waitFor([&] {
        const auto status = service.getStatusSnapshot();
        return status.ready && !status.configurationPending && status.generation > before.generation;
    });
    const auto after = service.getStatusSnapshot();
    service.stopTasks();
    TEST_ASSERT_TRUE(ready && applied && revalidated);
    TEST_ASSERT_EQUAL_UINT32(before.generation, unchanged.generation);
    TEST_ASSERT_EQUAL_UINT32(before.reconnectAttempts, after.reconnectAttempts);
    TEST_ASSERT_EQUAL_UINT32(0, after.fifoResyncs);
    TEST_ASSERT_EQUAL_UINT32(0, after.irqFallbacks);
}
