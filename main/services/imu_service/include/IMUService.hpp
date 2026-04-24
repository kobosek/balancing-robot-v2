#pragma once

#include "EventHandler.hpp"
#include "IIMUFaultSink.hpp"
#include "IMUState.hpp"
#include "MPU6050Profile.hpp"
#include "config/MPU6050Config.hpp"
#include "config/SystemBehaviorConfig.hpp"
#include "esp_err.h"
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>

class BaseEvent;
class CONFIG_BehaviorConfigUpdate;
class CONFIG_ImuConfigUpdate;
class FIFOProcessor;
class FIFOTask;
class HealthMonitorTask;
class I2CDevice;
class IMUCalibration;
class IMUHealthMonitor;
class IMU_AttachRequested;
class IMU_CalibrationRequest;
class IMU_SystemPolicyChanged;
class MPU6050Driver;
class MPU6050HardwareController;
class OrientationEstimator;
class EventBus;

class IMUService : public EventHandler, public IIMUFaultSink {
public:
    IMUService(std::shared_ptr<OrientationEstimator> estimator,
               const MPU6050Config& config,
               const SystemBehaviorConfig& behaviorConfig,
               EventBus& bus);
    ~IMUService();

    esp_err_t init();
    bool startTasks();
    void stopTasks();

    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }
    void onIMUHardFault(esp_err_t errorCode) override;
    void pollBackgroundMaintenance();

    IMUState getCurrentState() const;
    bool isAvailable() const;

    static const char* stateToString(IMUState state);

private:
    static constexpr const char* TAG = "IMUService";

    std::unique_ptr<I2CDevice> m_i2cDevice;
    std::unique_ptr<MPU6050Driver> m_driver;
    std::unique_ptr<MPU6050HardwareController> m_hardwareController;
    std::shared_ptr<OrientationEstimator> m_estimator;
    MPU6050Config m_config;
    SystemBehaviorConfig m_behaviorConfig;
    EventBus& m_eventBus;

    std::unique_ptr<IMUHealthMonitor> m_healthMonitor;
    std::unique_ptr<FIFOProcessor> m_fifoProcessor;
    std::unique_ptr<IMUCalibration> m_calibration;
    std::unique_ptr<FIFOTask> m_fifoTask;
    std::unique_ptr<HealthMonitorTask> m_healthMonitorTask;

    std::atomic<bool> m_is_calibrating_flag;
    IMUState m_current_state;
    std::atomic<bool> m_calibration_allowed;
    std::atomic<bool> m_auto_attach_allowed;
    std::atomic<bool> m_hardware_config_apply_allowed;
    std::atomic<bool> m_pending_hardware_apply;
    std::atomic<bool> m_fault_reported;
    std::atomic<int64_t> m_next_auto_attach_time_us;
    std::atomic<bool> m_initialized;
    MPU6050Profile m_profile;

    mutable std::mutex m_state_mutex;
    mutable std::mutex m_config_mutex;
    mutable std::mutex m_attach_mutex;

    bool transitionToState(IMUState newState);
    esp_err_t enterInitializedState();
    esp_err_t exitInitializedState();
    esp_err_t enterOperationalState();
    esp_err_t exitOperationalState();
    esp_err_t enterCalibrationState();
    esp_err_t exitCalibrationState();
    esp_err_t enterUnavailableState();
    esp_err_t exitUnavailableState();
    static bool isValidTransition(IMUState from, IMUState to);

    void refreshDerivedStateLocked();
    void applyRuntimeProfile(const MPU6050Config& config,
                             const MPU6050Profile& profile,
                             bool reinitializeEstimator);
    bool applyConfig(const MPU6050Config& newConfig);
    void applyConfig(const SystemBehaviorConfig& config);
    bool canApplyHardwareConfigNow() const;
    void applyPendingHardwareConfigIfSafe();
    void publishAvailabilityIfOperational(bool shouldPublishAvailabilityEvent);
    esp_err_t attachAndConfigureCurrentProfile(bool publishAvailabilityEvent);
    esp_err_t attachAndConfigureCurrentProfileLocked(bool publishAvailabilityEvent,
                                                     bool* shouldPublishAvailabilityEvent = nullptr);
    void markSensorUnavailable(esp_err_t errorCode, bool publishErrorEvent);
    esp_err_t performCalibration();
    void scheduleAutoAttachRetry(int64_t delayUs = 0);

    void handleIMUConfigUpdate(const CONFIG_ImuConfigUpdate& event);
    void handleBehaviorConfigUpdate(const CONFIG_BehaviorConfigUpdate& event);
    void handleCalibrationRequest(const IMU_CalibrationRequest& event);
    void handleAttachRequested(const IMU_AttachRequested& event);
    void handleSystemPolicyChanged(const IMU_SystemPolicyChanged& event);

    void safeConfigUpdate(const std::function<void()>& updateFunc);
};
