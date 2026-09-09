#pragma once
#include "EventHandler.hpp"
#include "IMUState.hpp"
#include "IMUDataTypes.hpp"
#include "MPU6050Profile.hpp"
#include "IMUDataReadyInterrupt.hpp"
#include "config/SystemBehaviorConfig.hpp"
#include <atomic>
#include <memory>
#include <mutex>
class EventBus;
class I2CDevice;
class MPU6050Driver;
class MPU6050HardwareController;
class FIFOProcessor;
class IMUCalibration;
class OrientationEstimator;
class IMUTask;
class IMUService : public EventHandler {
public:
    IMUService(std::shared_ptr<OrientationEstimator>, const MPU6050Config&, const SystemBehaviorConfig&, EventBus&);
    ~IMUService();
    esp_err_t init();
    bool startTasks();
    void stopTasks();
    void handleEvent(const BaseEvent&) override;
    std::string getHandlerName() const override { return "IMUService"; }
    IMUState getCurrentState() const;
    bool isAvailable() const;
    IMUStatusSnapshot getStatusSnapshot() const;
    bool reserveMotion(uint32_t& generation, const char** rejectionReason = nullptr);
    void releaseMotion();
    bool reserveOta();
    void releaseOta();
    void runWorker(IMUTask&);
    static const char* stateToString(IMUState);
private:
    static constexpr const char* TAG = "IMUService";
    EventBus& m_bus;
    std::shared_ptr<OrientationEstimator> m_estimator;
    std::unique_ptr<I2CDevice> m_device;
    std::unique_ptr<MPU6050Driver> m_driver;
    std::unique_ptr<MPU6050HardwareController> m_hardware;
    std::unique_ptr<FIFOProcessor> m_fifo;
    std::unique_ptr<IMUCalibration> m_calibration;
    std::unique_ptr<IMUTask> m_task;
    IMUDataReadyInterrupt m_irq;
    mutable std::mutex m_mutex;
    MPU6050Config m_desired, m_applied;
    SystemBehaviorConfig m_behavior;
    IMUStatusSnapshot m_status;
    IMUState m_state = IMUState::INITIALIZED;
    bool m_otaReserved = false;
    std::atomic<bool> m_cancelCalibration{false};
    bool m_motionReserved = false, m_calibrationPending = false;
    bool m_applyAllowed = false, m_attachAllowed = false;
    std::atomic<bool> m_calibrationAllowed{false};
    bool m_configPending = true, m_initialized = false;
    // Worker-owned state below.
    MPU6050Profile m_profile;
    int64_t m_nextAttemptUs = 0, m_validationDeadlineUs = 0, m_lastProgressUs = 0;
    unsigned m_validationSamples = 0;
    bool m_resyncUsed = false, m_incident = false, m_fullAttachValidating = false;
    void transition(IMUState, esp_err_t = ESP_OK, IMUFaultReason = IMUFaultReason::NONE);
    void invalidate(IMUFaultReason);
    void unavailable(esp_err_t, IMUFaultReason);
    esp_err_t attach(const MPU6050Config&, IMUTask&);
    void applySoftwareConfiguration(const MPU6050Config&);
    esp_err_t startStream(IMUTask&);
    void calibrate(IMUTask&);
};
