#include "SPIFFSStorageService.hpp"     // Relative path within module's include dir
#include "esp_spiffs.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <cstdio>                       // For C-style file I/O
#include <sys/stat.h>                   // For checking file existence/size (alternative)
#include <cerrno>                       // Use <cerrno> for errno C++ style
#include <cstring>                      // For strerror C++ style
#include <new>                          // For std::nothrow
#include <algorithm>
#include <cstdint>

namespace {
constexpr const char* CONFIG_BACKUP_NAMESPACE = "ota_cfg";
constexpr const char* CONFIG_BACKUP_PENDING_KEY = "pending";
constexpr size_t CONFIG_BACKUP_CHUNK_SIZE = 1024;
constexpr size_t CONFIG_BACKUP_MAX_BYTES = 16384;

esp_err_t ensureNvs()
{
    const esp_err_t ret = nvs_flash_init();
    if (ret == ESP_OK || ret == ESP_ERR_NVS_INVALID_STATE) return ESP_OK;
    return ret;
}
}

SPIFFSStorageService::SPIFFSStorageService(const char* partition_label, const char* base_path, size_t max_files) :
    m_partition_label(partition_label),
    m_base_path(base_path),
    m_max_files(max_files),
    m_initialized(false),
    m_recoveryRequired(false)
{}

bool SPIFFSStorageService::partitionLooksErased() const
{
    const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
        m_partition_label);
    if (!partition) return false;

    uint8_t probe[64] = {};
    if (esp_partition_read(partition, 0, probe, sizeof(probe)) != ESP_OK) {
        // Unknown is treated as non-erased: never turn an I/O failure into an
        // implicit format operation.
        return false;
    }
    bool allErased = true;
    bool allZero = true;
    for (uint8_t byte : probe) {
        allErased = allErased && byte == 0xFF;
        allZero = allZero && byte == 0x00;
    }
    return allErased || allZero;
}

esp_err_t SPIFFSStorageService::init() {
    if (m_initialized) {
        ESP_LOGI(TAG, "SPIFFS already initialized.");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing SPIFFS partition: %s, base_path: %s", m_partition_label, m_base_path);
    const bool firstBootPartition = partitionLooksErased();
    // Publish the probe result before attempting the mount.  If an existing
    // medium cannot be mounted, ConfigurationService must distinguish that
    // recovery case from a genuinely erased first boot and must never create
    // defaults silently.
    m_firstBootPartition = firstBootPartition;
    esp_vfs_spiffs_conf_t conf = { // Now the type is fully known
        .base_path = m_base_path,
        .partition_label = m_partition_label,
        .max_files = m_max_files,
        // Formatting is allowed only for a demonstrably erased partition.
        // A mount failure on an existing filesystem is a recovery condition;
        // silently formatting it would destroy the only configuration copy.
        .format_if_mount_failed = firstBootPartition
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
        if (!firstBootPartition && ret == ESP_FAIL) {
            m_recoveryRequired = true;
            ESP_LOGE(TAG, "Existing SPIFFS could not be mounted; preserving it and requiring recovery");
            return ESP_OK;
        }
        return ret; // Return specific error for first-boot/partition failures
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
    const esp_err_t restoreRet = restorePendingOtaBackup();
    if (restoreRet != ESP_OK && restoreRet != ESP_ERR_NOT_FOUND) {
        m_recoveryRequired = true;
        ESP_LOGE(TAG, "Pending OTA configuration could not be restored: %s",
                 esp_err_to_name(restoreRet));
        // Keep the filesystem mounted for diagnostics, but make all future
        // reads/writes fail closed so ConfigurationService latches recovery.
        return ESP_OK;
    }
    ESP_LOGI(TAG, "SPIFFS initialized successfully.");
    return ESP_OK;
}

esp_err_t SPIFFSStorageService::loadData(const std::string& key, std::string& data) {
    if (!m_initialized || m_recoveryRequired) {
        ESP_LOGE(TAG, "SPIFFS not initialized, cannot load data.");
        return ESP_ERR_INVALID_STATE;
    }

    const std::string full_path = std::string(m_base_path) + "/" + key;
    const std::string backup_path = full_path + ".bak";
    ESP_LOGD(TAG, "Loading data from: %s", full_path.c_str());

    FILE* f = fopen(full_path.c_str(), "r");
    if (f == NULL) {
        if (errno == ENOENT) {
            // Recover the last complete record if power was lost after the
            // old file was moved aside but before the replacement was
            // published.
            if (std::rename(backup_path.c_str(), full_path.c_str()) == 0) {
                f = fopen(full_path.c_str(), "r");
            }
            if (f == NULL && errno == ENOENT) {
                ESP_LOGW(TAG, "File not found: %s", full_path.c_str());
                return ESP_ERR_NOT_FOUND;
            }
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to recover configuration file: %s (errno %d)",
                         full_path.c_str(), errno);
                return ESP_FAIL;
            }
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
     if (!m_initialized || m_recoveryRequired) {
        ESP_LOGE(TAG, "SPIFFS not initialized, cannot save data.");
        return ESP_ERR_INVALID_STATE;
    }

    const std::string full_path = std::string(m_base_path) + "/" + key;
    const std::string temp_path = full_path + ".tmp";
    const std::string backup_path = full_path + ".bak";
    ESP_LOGD(TAG, "Saving data to temporary path: %s (%zu bytes)", temp_path.c_str(), data.length());

    // Keep the last complete record in place until the replacement has been
    // fully written and closed. This prevents a short write or a reset during
    // fwrite from turning the only configuration file into a truncated file.
    FILE* f = fopen(temp_path.c_str(), "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open temporary file for writing: %s (errno %d)", temp_path.c_str(), errno);
        return ESP_FAIL;
    }

    size_t written_size = 0;
    if (!data.empty()) {
        written_size = fwrite(data.c_str(), 1, data.length(), f);
    }

    if (ferror(f)) {
        ESP_LOGE(TAG, "Error occurred during fwrite to %s", temp_path.c_str());
        fclose(f); // Attempt to close anyway
        std::remove(temp_path.c_str());
        return ESP_FAIL;
    }

    if (fclose(f) != 0) {
         ESP_LOGE(TAG, "Failed to close temporary file after writing: %s (errno %d)", temp_path.c_str(), errno);
         std::remove(temp_path.c_str());
         return ESP_FAIL;
    }

    if (written_size != data.length()) {
        ESP_LOGE(TAG, "Failed to write full data content to %s (wrote %zu/%zu bytes)", temp_path.c_str(), written_size, data.length());
        std::remove(temp_path.c_str());
        return ESP_FAIL;
    }

    // SPIFFS refuses to rename over an existing object. Move the previous
    // complete record aside first, then publish the new one. If publication
    // fails, restore the previous name before returning an error. When the
    // main file is already missing, keep an existing backup intact so a
    // previous interrupted transaction remains recoverable on the next boot.
    const auto pathExists = [](const std::string& path, bool& exists) -> esp_err_t {
        struct stat info = {};
        if (stat(path.c_str(), &info) == 0) {
            exists = true;
            return ESP_OK;
        }
        if (errno == ENOENT) {
            exists = false;
            return ESP_OK;
        }
        return ESP_FAIL;
    };

    bool fullExists = false;
    bool backupExists = false;
    if (pathExists(full_path, fullExists) != ESP_OK ||
        pathExists(backup_path, backupExists) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to inspect configuration transaction files");
        std::remove(temp_path.c_str());
        return ESP_FAIL;
    }

    bool hadPrevious = false;
    if (fullExists) {
        if (backupExists && std::remove(backup_path.c_str()) != 0 && errno != ENOENT) {
            ESP_LOGE(TAG, "Failed to remove stale configuration backup %s (errno %d)",
                     backup_path.c_str(), errno);
            std::remove(temp_path.c_str());
            return ESP_FAIL;
        }
        if (std::rename(full_path.c_str(), backup_path.c_str()) != 0) {
            ESP_LOGE(TAG, "Failed to protect previous configuration %s (errno %d)",
                     full_path.c_str(), errno);
            std::remove(temp_path.c_str());
            return ESP_FAIL;
        }
        hadPrevious = true;
    }

    if (std::rename(temp_path.c_str(), full_path.c_str()) != 0) {
        ESP_LOGE(TAG, "Failed to publish configuration file %s (errno %d)", full_path.c_str(), errno);
        if (hadPrevious) {
            if (std::rename(backup_path.c_str(), full_path.c_str()) != 0) {
                ESP_LOGE(TAG, "Failed to restore previous configuration %s (errno %d)",
                         full_path.c_str(), errno);
            }
        }
        std::remove(temp_path.c_str());
        return ESP_FAIL;
    }
    // Keep the previous complete record until the caller verifies the new
    // bytes and explicitly commits them.  This gives ConfigurationService a
    // real rollback source when a readback fails after publication.
    ESP_LOGD(TAG, "Data saved successfully to %s; previous record retained until commit",
             full_path.c_str());
    return ESP_OK;
}

esp_err_t SPIFFSStorageService::commitData(const std::string& key)
{
    if (!m_initialized || m_recoveryRequired) return ESP_ERR_INVALID_STATE;
    const std::string backupPath =
        std::string(m_base_path) + "/" + key + ".bak";
    if (std::remove(backupPath.c_str()) == 0 || errno == ENOENT) {
        return ESP_OK;
    }
    ESP_LOGW(TAG, "Failed to remove committed backup %s (errno %d)",
             backupPath.c_str(), errno);
    return ESP_FAIL;
}

esp_err_t SPIFFSStorageService::restoreBackup(const std::string& key)
{
    if (!m_initialized || m_recoveryRequired) return ESP_ERR_INVALID_STATE;
    const std::string fullPath = std::string(m_base_path) + "/" + key;
    const std::string backupPath = fullPath + ".bak";
    const std::string failedPath = fullPath + ".failed";

    struct stat backupInfo = {};
    if (stat(backupPath.c_str(), &backupInfo) != 0) {
        return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
    }

    // Rename the unverified current record out of the way.  If publishing the
    // old record fails, put that current record back so recovery can be
    // retried without silently deleting either version.
    (void)std::remove(failedPath.c_str());
    bool movedCurrent = false;
    struct stat currentInfo = {};
    if (stat(fullPath.c_str(), &currentInfo) == 0) {
        if (std::rename(fullPath.c_str(), failedPath.c_str()) != 0) {
            return ESP_FAIL;
        }
        movedCurrent = true;
    } else if (errno != ENOENT) {
        return ESP_FAIL;
    }

    if (std::rename(backupPath.c_str(), fullPath.c_str()) != 0) {
        if (movedCurrent) (void)std::rename(failedPath.c_str(), fullPath.c_str());
        return ESP_FAIL;
    }
    if (movedCurrent && std::remove(failedPath.c_str()) != 0 && errno != ENOENT) {
        ESP_LOGW(TAG, "Restored %s but could not remove failed record (errno %d)",
                 fullPath.c_str(), errno);
    }
    ESP_LOGW(TAG, "Restored previous complete record for %s", key.c_str());
    return ESP_OK;
}

esp_err_t SPIFFSStorageService::restorePendingOtaBackup()
{
    const esp_err_t nvsInit = ensureNvs();
    if (nvsInit != ESP_OK) return nvsInit;

    nvs_handle_t handle = 0;
    esp_err_t ret = nvs_open(CONFIG_BACKUP_NAMESPACE, NVS_READWRITE, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) return ESP_ERR_NOT_FOUND;
    if (ret != ESP_OK) return ret;
    uint8_t pending = 0;
    ret = nvs_get_u8(handle, CONFIG_BACKUP_PENDING_KEY, &pending);
    if (ret == ESP_ERR_NVS_NOT_FOUND || pending == 0) {
        nvs_close(handle);
        return ESP_ERR_NOT_FOUND;
    }
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    const auto readBlob = [&](const char* prefix, std::string& output) -> esp_err_t {
        output.clear();
        char lengthKey[16] = {};
        char countKey[16] = {};
        std::snprintf(lengthKey, sizeof(lengthKey), "%s_len", prefix);
        std::snprintf(countKey, sizeof(countKey), "%s_cnt", prefix);
        uint32_t length = 0;
        uint32_t count = 0;
        esp_err_t result = nvs_get_u32(handle, lengthKey, &length);
        // A pending backup must contain an explicit length/count pair.  A
        // missing key means the NVS record itself is torn, even when the
        // corresponding file was legitimately empty on first boot.
        if (result == ESP_ERR_NVS_NOT_FOUND) return ESP_ERR_INVALID_STATE;
        if (result != ESP_OK) return result;
        result = nvs_get_u32(handle, countKey, &count);
        if (result != ESP_OK) return result;
        const uint32_t expectedCount = static_cast<uint32_t>(
            (static_cast<size_t>(length) + CONFIG_BACKUP_CHUNK_SIZE - 1) /
            CONFIG_BACKUP_CHUNK_SIZE);
        if (length > CONFIG_BACKUP_MAX_BYTES || count != expectedCount || count > 32) {
            return ESP_ERR_INVALID_SIZE;
        }
        if (length > 0) output.resize(length);
        size_t offset = 0;
        for (uint32_t index = 0; index < count; ++index) {
            char key[16] = {};
            std::snprintf(key, sizeof(key), "%s%u", prefix,
                          static_cast<unsigned>(index));
            const size_t expected = std::min(CONFIG_BACKUP_CHUNK_SIZE,
                                              static_cast<size_t>(length) - offset);
            size_t actual = expected;
            result = nvs_get_blob(handle, key,
                                  length > 0 ? &output[offset] : nullptr,
                                  &actual);
            if (result != ESP_OK || actual != expected) {
                return result == ESP_OK ? ESP_FAIL : result;
            }
            offset += actual;
        }
        return ESP_OK;
    };

    std::string config;
    std::string operation;
    ret = readBlob("cfg", config);
    if (ret == ESP_OK) ret = readBlob("op", operation);
    if (ret == ESP_OK && !config.empty()) {
        ret = saveData("config.json", config);
        if (ret == ESP_OK) ret = commitData("config.json");
    }
    if (ret == ESP_OK) {
        if (!operation.empty()) {
            ret = saveData("config.json.operation", operation);
            if (ret == ESP_OK) ret = commitData("config.json.operation");
        } else {
            // The backup explicitly recorded an absent/empty sidecar. Remove
            // any stale journal left by an interrupted restore before
            // treating the NVS record as consumed.
            const std::string operationPath =
                std::string(m_base_path) + "/config.json.operation";
            if (std::remove(operationPath.c_str()) != 0 && errno != ENOENT) {
                ret = ESP_FAIL;
            }
        }
    }
    if (ret == ESP_OK) {
        ret = nvs_erase_all(handle);
        if (ret == ESP_OK) ret = nvs_commit(handle);
    }
    nvs_close(handle);
    if (ret != ESP_OK) return ret;
    ESP_LOGI(TAG, "Restored pending OTA configuration backup");
    return ESP_OK;
}
