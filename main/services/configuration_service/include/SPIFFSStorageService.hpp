#pragma once
#include <string>
#include "esp_err.h"

class IStorageService {
public:
    virtual ~IStorageService() = default;
    virtual esp_err_t init() = 0;
    virtual esp_err_t loadData(const std::string& key, std::string& data) = 0;
    virtual esp_err_t saveData(const std::string& key, const std::string& data) = 0;
};

class SPIFFSStorageService : public IStorageService { // Inherit from interface if kept
public:
    SPIFFSStorageService(const char* partition_label = "storage", const char* base_path = "/spiffs", size_t max_files = 5);
    ~SPIFFSStorageService() override = default;

    esp_err_t init() override;
    esp_err_t loadData(const std::string& key, std::string& data) override;
    esp_err_t saveData(const std::string& key, const std::string& data) override;

private:
    static constexpr const char* TAG = "SPIFFSStorage";
    const char* m_partition_label;
    const char* m_base_path;
    size_t m_max_files;
    bool m_initialized = false;
};