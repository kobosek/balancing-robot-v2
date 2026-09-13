#pragma once
#include <string>
#include "esp_err.h"

class IStorageService {
public:
    virtual ~IStorageService() = default;
    virtual esp_err_t init() = 0;
    // A missing record is only a normal first-boot condition when the
    // backend can identify its medium as genuinely fresh.  Backends that do
    // not have a physical medium (for example host-test memory stores) keep
    // the historical default; SPIFFS overrides this after probing its
    // partition before mounting.
    virtual bool isFreshStorage() const { return true; }
    virtual esp_err_t loadData(const std::string& key, std::string& data) = 0;
    virtual esp_err_t saveData(const std::string& key, const std::string& data) = 0;
    // A transactional writer may keep the previous complete record beside a
    // newly published file until readback succeeds.  Backends without a
    // separate backup namespace retain the no-op/default behavior.
    virtual esp_err_t commitData(const std::string& key) { return ESP_OK; }
    virtual esp_err_t restoreBackup(const std::string& key) {
        (void)key;
        return ESP_ERR_NOT_SUPPORTED;
    }
};

class SPIFFSStorageService : public IStorageService { // Inherit from interface if kept
public:
    SPIFFSStorageService(const char* partition_label = "storage", const char* base_path = "/spiffs", size_t max_files = 5);
    ~SPIFFSStorageService() override = default;

    esp_err_t init() override;
    esp_err_t loadData(const std::string& key, std::string& data) override;
    esp_err_t saveData(const std::string& key, const std::string& data) override;
    esp_err_t commitData(const std::string& key) override;
    esp_err_t restoreBackup(const std::string& key) override;
    bool isFreshStorage() const override { return m_firstBootPartition; }
    bool recoveryRequired() const { return m_recoveryRequired; }

private:
    static constexpr const char* TAG = "SPIFFSStorage";
    const char* m_partition_label;
    const char* m_base_path;
    size_t m_max_files;
    bool m_initialized = false;
    bool m_recoveryRequired = false;
    bool m_firstBootPartition = true;

    bool partitionLooksErased() const;
    esp_err_t restorePendingOtaBackup();
};
