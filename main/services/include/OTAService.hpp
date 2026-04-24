#pragma once

#include "EventHandler.hpp"
#include "esp_err.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include <mutex>
#include <string>

enum class OTAUpdateTarget {
    APP,
    SPIFFS
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
};

class OTAService : public EventHandler {
public:
    esp_err_t init();

    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

    OTAStatus getStatus() const;
    esp_err_t begin(size_t expectedSize);
    esp_err_t beginAppUpdate(size_t expectedSize);
    esp_err_t beginSpiffsUpdate(size_t expectedSize);
    esp_err_t write(const uint8_t* data, size_t len);
    esp_err_t finish();
    void abort();

private:
    static constexpr const char* TAG = "OTAService";
    static constexpr const char* SPIFFS_PARTITION_LABEL = "storage";

    mutable std::mutex m_mutex;
    const esp_partition_t* m_runningPartition = nullptr;
    const esp_partition_t* m_updatePartition = nullptr;
    const esp_partition_t* m_spiffsPartition = nullptr;
    esp_ota_handle_t m_updateHandle = 0;
    OTAUpdateTarget m_activeTarget = OTAUpdateTarget::APP;
    size_t m_writeOffset = 0;
    bool m_bundleSpiffsReady = false;
    OTAStatus m_status;

    esp_err_t beginAppUpdateLocked(size_t expectedSize);
    esp_err_t beginSpiffsUpdateLocked(size_t expectedSize);
    esp_err_t writeAppLocked(const uint8_t* data, size_t len);
    esp_err_t writeSpiffsLocked(const uint8_t* data, size_t len);
    esp_err_t finishAppLocked();
    esp_err_t finishSpiffsLocked();
    void setUpdateAllowed(bool allowed);
};
