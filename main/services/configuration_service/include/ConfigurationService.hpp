#pragma once

#include "ConfigData.hpp"
#include "ConfigChangePublisher.hpp"
#include "ConfigValidator.hpp"
#include "CONTROL_RunModeChanged.hpp"
#include "ControlOperationGate.hpp"
#include "EventBus.hpp"
#include "EventHandler.hpp"
#include <string>
#include <mutex>
#include <cstdint>
#include "esp_err.h" // Include ESP types

class IStorageService;
class IConfigParser;
class ConfigUpdatedEvent;
class BaseEvent;
class IMU_GyroOffsetsUpdated;

class ConfigurationService : public EventHandler {
public:
    ConfigurationService(IStorageService& storage,
                         IConfigParser& parser,
                         EventBus& bus,
                         const std::string& configKey = "config.json",
                         ControlOperationGate* operationGate = nullptr);

    esp_err_t init();
    esp_err_t save();
    esp_err_t updateConfigFromJson(const std::string& json, std::string* error = nullptr);
    // A tuning candidate supplies the document revision it was based on. The
    // default preserves existing callers while allowing candidate saves to
    // reject a stale full-document snapshot.
    esp_err_t applyPidConfig(const std::string& pidName,
                             const PIDConfig& config,
                             bool persist,
                             uint64_t expectedConfigRevision = UINT64_MAX);
    esp_err_t getJsonString(std::string& jsonOutput) const;
    // The most recently accepted full-document operation.  The identifier is
    // stable for the lifetime of the in-memory service so a client can safely
    // retry after losing an HTTP response.
    uint64_t getLastOperationId() const;
    ControlOperationStatus getOperationStatus() const;
    // A reset can interrupt the short interval between publishing a saved
    // document and recording the committed operation. Keep the robot
    // disarmed until the client explicitly reconciles that operation.
    bool isOperationRecoveryPending() const;
    esp_err_t acknowledgeOperation(uint64_t operationId);

    // Getters
    ConfigData getConfigData() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData; }
    WiFiConfig getWiFiConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.wifi; }
    PIDConfig getPidAngleConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.control.strategies.nested_pid.angle; }
    PIDConfig getPidSpeedLeftConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.control.strategies.nested_pid.speed_left; }
    PIDConfig getPidSpeedRightConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.control.strategies.nested_pid.speed_right; }
    PIDConfig getPidYawAngleConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.control.strategies.nested_pid.yaw_angle; }
    PIDConfig getPidYawRateConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.control.strategies.nested_pid.yaw_rate; }
    PidTuningConfig getPidTuningConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.pid_tuning; }
    MPU6050Config getMpu6050Config() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.imu; }
    MainLoopConfig getMainLoopConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.mainLoop; }
    EncoderConfig getEncoderConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.encoder; }
    MotorConfig getMotorConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.motor; }
    BatteryConfig getBatteryConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.battery; }
    ControlConfig getControlConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.control; }
    BalanceStrategiesConfig getBalanceStrategiesConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.control.strategies; }
    // --- Getters for NEW sections ---
    SystemBehaviorConfig getSystemBehaviorConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.behavior; }
    RobotDimensionsConfig getRobotDimensionsConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.dimensions; }
    WebServerConfig getWebServerConfig() const { std::lock_guard<std::mutex> lock(m_mutex); return m_configData.web; }

    // Update persistent gyro offsets
    void updateImuGyroOffsets(float x, float y, float z);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }
    
    // Keep for backward compatibility
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "ConfigService";
    IStorageService& m_storageService;
    IConfigParser& m_configParser;
    EventBus& m_eventBus;
    const std::string m_configKey;
    const std::string m_operationJournalKey;
    ControlOperationGate* m_operationGate = nullptr;
    ConfigValidator m_configValidator;
    ConfigChangePublisher m_configChangePublisher;
    ConfigData m_configData; // Holds the current configuration
    mutable std::mutex m_mutex; // Protects m_configData
    // Serializes read/modify/write operations and their event publication so
    // a second save cannot publish an older snapshot after a newer one.
    mutable std::mutex m_operationMutex;
    mutable std::mutex m_operationStatusMutex;
    uint64_t m_localOperationSequence = 0;
    uint64_t m_activeOperationId = 0;
    ControlOperationPhase m_activeOperationPhase = ControlOperationPhase::IDLE;
    uint64_t m_lastOperationId = 0;
    uint64_t m_lastOperationFingerprint = 0;
    esp_err_t m_lastOperationResult = ESP_ERR_INVALID_STATE;
    ControlOperationPhase m_lastOperationPhase = ControlOperationPhase::IDLE;
    uint32_t m_lastOperationBaseRevision = 0;
    uint32_t m_lastOperationTargetRevision = 0;
    bool m_hasLastOperation = false;
    bool m_operationRecoveryPending = false;
    uint64_t m_recoveryOperationId = 0;
    uint64_t m_recoveryOperationFingerprint = 0;
    uint32_t m_recoveryBaseRevision = 0;
    uint32_t m_recoveryTargetRevision = 0;
    uint64_t m_activeOperationFingerprint = 0;
    uint32_t m_activeOperationBaseRevision = 0;
    uint32_t m_activeOperationTargetRevision = 0;
    // Set by saveInternal when a failed persistence attempt is proven not to
    // have changed the durable document (or the previous record was restored
    // and verified).  The operation completion path uses this to distinguish
    // a resolved write failure from an uncertain transaction that must keep
    // the recovery latch asserted.
    bool m_lastPersistenceFailureResolved = false;
    bool m_controlActive = false;
    uint64_t m_controlArmId = 0;

    // Serialize and persist a prepared snapshot without holding the runtime
    // configuration mutex. The operation mutex keeps this I/O exclusive;
    // callers publish/commit the snapshot only after storage verification.
    esp_err_t saveInternal(const ConfigData& snapshot);
    esp_err_t persistOperationJournal(uint64_t operationId,
                                      uint64_t fingerprint,
                                      uint32_t baseRevision,
                                      uint32_t targetRevision,
                                      ControlOperationPhase phase,
                                      int32_t resultCode);
    esp_err_t loadOperationJournal();
    uint64_t beginConfigurationOperation(
        const ControlOperationReservation& reservation,
        uint64_t fingerprint,
        uint32_t baseRevision,
        uint32_t targetRevision,
        uint64_t requestedOperationId = 0);
    esp_err_t completeConfigurationOperation(
        ControlOperationReservation& reservation,
        uint64_t operationId,
        uint64_t fingerprint,
        uint32_t baseRevision,
        uint32_t targetRevision,
        ControlOperationPhase phase,
        esp_err_t result);
    void setActiveOperationPhase(uint64_t operationId,
                                 ControlOperationPhase phase);
    void handleRunModeChanged(const CONTROL_RunModeChanged& event);
    bool tryReserveConfiguration(ControlOperationReservation& reservation,
                                 const char* operation,
                                 uint64_t requestedOperationId = 0);
};
