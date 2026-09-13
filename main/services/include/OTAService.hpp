#pragma once

#include "EventHandler.hpp"
#include "ControlOperationGate.hpp"
#include "esp_err.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include <cstdint>
#include <mutex>
#include <string>

enum class OTAUpdateTarget {
    APP,
    SPIFFS
};

// Durable two-part OTA progress.  The marker lives in its own NVS namespace
// so restoring the configuration backup cannot erase the bundle journal.
enum class OTABundleStage : uint8_t {
    NONE = 0,
    SPIFFS_WRITING,
    SPIFFS_READY,
    APP_WRITING,
    APP_PENDING_REBOOT,
    RECOVERY
};

struct OTAStatus {
    bool available = false;
    bool spiffsAvailable = false;
    bool updateAllowed = false;
    bool updateInProgress = false;
    bool rebootRequired = false;
    size_t bytesWritten = 0;
    size_t expectedSize = 0;
    size_t spiffsPartitionSize = 0;
    std::string runningPartition;
    std::string updatePartition;
    std::string spiffsPartition;
    std::string appVersion;
    std::string activeTarget;
    std::string message;
    // A decimal-safe identifier and stage let a client distinguish a normal
    // idle OTA service from a bundle that needs explicit recovery after reset.
    uint64_t bundleId = 0;
    std::string bundleStage = "none";
    bool bundleRecoveryPending = false;
};

class IMUService;
class OTAService : public EventHandler {
public:
    esp_err_t init();
    void bindImu(IMUService& imu) { m_imu = &imu; }
    void bindOperationGate(ControlOperationGate& operationGate) {
        m_operationGate = &operationGate;
    }

    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

    OTAStatus getStatus() const;
    ControlOperationStatus getOperationStatus() const;
    esp_err_t begin(size_t expectedSize);
    esp_err_t beginAppUpdate(size_t expectedSize);
    esp_err_t beginSpiffsUpdate(size_t expectedSize);
    esp_err_t write(const uint8_t* data, size_t len);
    esp_err_t finish();
    void abort();

private:
    static constexpr const char* TAG = "OTAService";
    static constexpr const char* SPIFFS_PARTITION_LABEL = "storage";

    IMUService* m_imu = nullptr;
    ControlOperationGate* m_operationGate = nullptr;
    ControlOperationReservation m_operationReservation;
    mutable std::mutex m_mutex;
    const esp_partition_t* m_runningPartition = nullptr;
    const esp_partition_t* m_updatePartition = nullptr;
    const esp_partition_t* m_spiffsPartition = nullptr;
    esp_ota_handle_t m_updateHandle = 0;
    OTAUpdateTarget m_activeTarget = OTAUpdateTarget::APP;
    size_t m_writeOffset = 0;
    bool m_bundleSpiffsReady = false;
    bool m_configurationBackupStored = false;
    OTABundleStage m_bundleStage = OTABundleStage::NONE;
    uint64_t m_bundleId = 0;
    size_t m_bundleSpiffsSize = 0;
    size_t m_bundleAppSize = 0;
    std::string m_bundleAppLabel;
    bool m_bundleRecoveryPending = false;
    OTAStatus m_status;

    esp_err_t beginAppUpdateLocked(size_t expectedSize);
    esp_err_t beginSpiffsUpdateLocked(size_t expectedSize);
    esp_err_t writeAppLocked(const uint8_t* data, size_t len);
    esp_err_t writeSpiffsLocked(const uint8_t* data, size_t len);
    esp_err_t finishAppLocked();
    esp_err_t finishSpiffsLocked();
    void setUpdateAllowed(bool allowed);
    bool tryReserveOtaLocked();
    void releaseOperationReservationLocked(
        ControlOperationPhase finalPhase = ControlOperationPhase::SUCCEEDED,
        int32_t resultCode = 0);
    esp_err_t backupConfigurationLocked();
    void clearConfigurationBackupLocked();
    esp_err_t loadBundleStateLocked();
    esp_err_t persistBundleStateLocked(OTABundleStage stage,
                                       uint64_t bundleId,
                                       size_t spiffsSize,
                                       size_t appSize,
                                       const char* appLabel);
    esp_err_t clearBundleStateLocked();
    void setBundleRecoveryPendingLocked(bool pending);
    void setBundleStageLocked(OTABundleStage stage,
                              uint64_t bundleId,
                              size_t spiffsSize,
                              size_t appSize,
                              const char* appLabel);
};
