#include "SPIFFSStorageService.hpp"     // Relative path within module's include dir
#include "esp_spiffs.h"
#include "esp_log.h"
#include <cstdio>                       // For C-style file I/O
#include <sys/stat.h>                   // For checking file existence/size (alternative)
#include <cerrno>                       // Use <cerrno> for errno C++ style
#include <cstring>                      // For strerror C++ style
#include <new>                          // For std::nothrow

SPIFFSStorageService::SPIFFSStorageService(const char* partition_label, const char* base_path, size_t max_files) :
    m_partition_label(partition_label),
    m_base_path(base_path),
    m_max_files(max_files),
    m_initialized(false) // Initialize state
{}

esp_err_t SPIFFSStorageService::init() {
    if (m_initialized) {
        ESP_LOGI(TAG, "SPIFFS already initialized.");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing SPIFFS partition: %s, base_path: %s", m_partition_label, m_base_path);
    esp_vfs_spiffs_conf_t conf = { // Now the type is fully known
        .base_path = m_base_path,
        .partition_label = m_partition_label,
        .max_files = m_max_files,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition '%s'", m_partition_label);
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret; // Return specific error
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(m_partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        // Continue initialization even if info fails
    } else {
        ESP_LOGI(TAG, "SPIFFS Partition size: total: %zu, used: %zu", total, used);
    }

    m_initialized = true; // Set flag only on success
    ESP_LOGI(TAG, "SPIFFS initialized successfully.");
    return ESP_OK;
}

esp_err_t SPIFFSStorageService::loadData(const std::string& key, std::string& data) {
    if (!m_initialized) {
        ESP_LOGE(TAG, "SPIFFS not initialized, cannot load data.");
        return ESP_ERR_INVALID_STATE;
    }

    std::string full_path = std::string(m_base_path) + "/" + key;
    ESP_LOGD(TAG, "Loading data from: %s", full_path.c_str());

    FILE* f = fopen(full_path.c_str(), "r");
    if (f == NULL) {
        if (errno == ENOENT) {
             ESP_LOGW(TAG, "File not found: %s", full_path.c_str());
             return ESP_ERR_NOT_FOUND;
        } else {
             ESP_LOGE(TAG, "Failed to open file for reading: %s (errno %d)", full_path.c_str(), errno);
             return ESP_FAIL;
        }
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size < 0) {
         ESP_LOGE(TAG, "Error determining file size: %s", full_path.c_str());
         fclose(f);
         return ESP_FAIL;
    }
    if (size == 0) {
        ESP_LOGD(TAG, "File is empty: %s", full_path.c_str());
        fclose(f);
        data.clear();
        return ESP_OK;
    }

    // --- Removed try-catch block ---
    // Rely on standard checks or behavior if allocation fails
    data.resize(size);
    // A simple check if resize actually worked (might not be foolproof without exceptions)
    if (data.capacity() < (size_t)size) {
         ESP_LOGE(TAG, "Failed to allocate memory (%ld bytes) for file buffer: %s", size, full_path.c_str());
         fclose(f);
         data.clear(); // Ensure data is cleared
         return ESP_ERR_NO_MEM;
    }

    size_t read_size = fread(&data[0], 1, size, f);
    fclose(f);

    if (read_size != (size_t)size) {
        ESP_LOGE(TAG, "Failed to read full file content from %s (read %zu/%ld bytes)", full_path.c_str(), read_size, size);
        data.clear();
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Data loaded successfully from %s (%ld bytes)", full_path.c_str(), size);
    return ESP_OK;
}

esp_err_t SPIFFSStorageService::saveData(const std::string& key, const std::string& data) {
     if (!m_initialized) {
        ESP_LOGE(TAG, "SPIFFS not initialized, cannot save data.");
        return ESP_ERR_INVALID_STATE;
    }

    std::string full_path = std::string(m_base_path) + "/" + key;
    ESP_LOGD(TAG, "Saving data to: %s (%zu bytes)", full_path.c_str(), data.length());

    FILE* f = fopen(full_path.c_str(), "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s (errno %d)", full_path.c_str(), errno);
        return ESP_FAIL;
    }

    size_t written_size = 0;
    if (!data.empty()) {
        written_size = fwrite(data.c_str(), 1, data.length(), f);
    }

    if (ferror(f)) {
        ESP_LOGE(TAG, "Error occurred during fwrite to %s", full_path.c_str());
        fclose(f); // Attempt to close anyway
        return ESP_FAIL;
    }

    if (fclose(f) != 0) {
         ESP_LOGE(TAG, "Failed to close file after writing (data may not be saved): %s (errno %d)", full_path.c_str(), errno);
         return ESP_FAIL;
    }

    if (written_size != data.length()) {
        ESP_LOGE(TAG, "Failed to write full data content to %s (wrote %zu/%zu bytes)", full_path.c_str(), written_size, data.length());
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Data saved successfully to %s", full_path.c_str());
    return ESP_OK;
}