#include "OTAService.hpp"

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_spiffs.h"

namespace {
std::string labelOrUnknown(const esp_partition_t* partition) {
    return partition ? std::string(partition->label) : std::string("unknown");
}
}

esp_err_t OTAService::init() {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_runningPartition = esp_ota_get_running_partition();
    m_updatePartition = esp_ota_get_next_update_partition(nullptr);
    m_spiffsPartition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
        SPIFFS_PARTITION_LABEL);

    const esp_app_desc_t* appDesc = esp_app_get_description();
    m_status.runningPartition = labelOrUnknown(m_runningPartition);
    m_status.updatePartition = labelOrUnknown(m_updatePartition);
    m_status.spiffsPartition = labelOrUnknown(m_spiffsPartition);
    m_status.appVersion = appDesc ? std::string(appDesc->version) : std::string("unknown");
    m_status.available = m_updatePartition != nullptr;
    m_status.spiffsAvailable = m_spiffsPartition != nullptr;
    m_status.updateInProgress = false;
    m_status.rebootRequired = false;
    m_status.bytesWritten = 0;
    m_status.expectedSize = 0;
    m_status.spiffsPartitionSize = m_spiffsPartition ? m_spiffsPartition->size : 0;
    m_status.activeTarget = "none";
    if (m_status.available && m_status.spiffsAvailable) {
        m_status.message = "OTA ready";
    } else if (m_status.available) {
        m_status.message = "App OTA ready; SPIFFS partition unavailable";
    } else if (m_status.spiffsAvailable) {
        m_status.message = "SPIFFS OTA ready; app OTA partition unavailable";
    } else {
        m_status.message = "No OTA-capable partitions available";
    }

    if (m_runningPartition != nullptr) {
        esp_ota_img_states_t otaState;
        if (esp_ota_get_state_partition(m_runningPartition, &otaState) == ESP_OK &&
            otaState == ESP_OTA_IMG_PENDING_VERIFY) {
            const esp_err_t markRet = esp_ota_mark_app_valid_cancel_rollback();
            if (markRet == ESP_OK) {
                ESP_LOGI(TAG, "Marked pending OTA image valid");
            } else {
                ESP_LOGW(TAG, "Failed to mark OTA image valid: %s", esp_err_to_name(markRet));
            }
        }
    }

    ESP_LOGI(TAG,
             "OTA init: app=%s spiffs=%s running=%s update=%s version=%s",
             m_status.available ? "true" : "false",
             m_status.spiffsAvailable ? "true" : "false",
             m_status.runningPartition.c_str(),
             m_status.updatePartition.c_str(),
             m_status.appVersion.c_str());
    return ESP_OK;
}

OTAStatus OTAService::getStatus() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_status;
}

esp_err_t OTAService::begin(size_t expectedSize) {
    return beginAppUpdate(expectedSize);
}

esp_err_t OTAService::beginAppUpdate(size_t expectedSize) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return beginAppUpdateLocked(expectedSize);
}

esp_err_t OTAService::beginSpiffsUpdate(size_t expectedSize) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return beginSpiffsUpdateLocked(expectedSize);
}

esp_err_t OTAService::beginAppUpdateLocked(size_t expectedSize) {
    if (m_status.updateInProgress) {
        return ESP_ERR_INVALID_STATE;
    }

    m_updatePartition = esp_ota_get_next_update_partition(nullptr);
    if (m_updatePartition == nullptr) {
        m_status.available = false;
        m_status.message = "No OTA update partition available";
        return ESP_ERR_NOT_FOUND;
    }

    const esp_err_t ret = esp_ota_begin(
        m_updatePartition,
        expectedSize > 0 ? expectedSize : OTA_SIZE_UNKNOWN,
        &m_updateHandle);
    if (ret != ESP_OK) {
        m_status.message = std::string("OTA begin failed: ") + esp_err_to_name(ret);
        return ret;
    }

    m_status.available = true;
    m_activeTarget = OTAUpdateTarget::APP;
    m_writeOffset = 0;
    m_status.updateInProgress = true;
    m_status.rebootRequired = false;
    m_status.bytesWritten = 0;
    m_status.expectedSize = expectedSize;
    m_status.activeTarget = "app";
    m_status.updatePartition = labelOrUnknown(m_updatePartition);
    m_status.message = "App OTA upload in progress";
    return ESP_OK;
}

esp_err_t OTAService::beginSpiffsUpdateLocked(size_t expectedSize) {
    if (m_status.updateInProgress) {
        return ESP_ERR_INVALID_STATE;
    }

    m_spiffsPartition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
        SPIFFS_PARTITION_LABEL);
    if (m_spiffsPartition == nullptr) {
        m_status.spiffsAvailable = false;
        m_status.message = "No SPIFFS partition available";
        return ESP_ERR_NOT_FOUND;
    }

    m_status.spiffsAvailable = true;
    m_status.spiffsPartition = labelOrUnknown(m_spiffsPartition);
    m_status.spiffsPartitionSize = m_spiffsPartition->size;
    if (expectedSize == 0 || expectedSize > m_spiffsPartition->size) {
        m_status.message = "SPIFFS image is empty or larger than storage partition";
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t ret = esp_vfs_spiffs_unregister(SPIFFS_PARTITION_LABEL);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPIFFS partition was not mounted before update");
        ret = ESP_OK;
    }
    if (ret != ESP_OK) {
        m_status.message = std::string("SPIFFS unmount failed: ") + esp_err_to_name(ret);
        return ret;
    }

    ret = esp_partition_erase_range(m_spiffsPartition, 0, m_spiffsPartition->size);
    if (ret != ESP_OK) {
        m_status.message = std::string("SPIFFS erase failed: ") + esp_err_to_name(ret);
        m_status.rebootRequired = true;
        return ret;
    }

    m_activeTarget = OTAUpdateTarget::SPIFFS;
    m_writeOffset = 0;
    m_status.updateInProgress = true;
    m_status.rebootRequired = false;
    m_status.bytesWritten = 0;
    m_status.expectedSize = expectedSize;
    m_status.activeTarget = "spiffs";
    m_status.message = "SPIFFS OTA upload in progress";
    return ESP_OK;
}

esp_err_t OTAService::write(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_status.updateInProgress || m_updateHandle == 0) {
        if (m_activeTarget != OTAUpdateTarget::SPIFFS) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    if (!m_status.updateInProgress) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return m_activeTarget == OTAUpdateTarget::SPIFFS
        ? writeSpiffsLocked(data, len)
        : writeAppLocked(data, len);
}

esp_err_t OTAService::writeAppLocked(const uint8_t* data, size_t len) {
    const esp_err_t ret = esp_ota_write(m_updateHandle, data, len);
    if (ret != ESP_OK) {
        m_status.message = std::string("OTA write failed: ") + esp_err_to_name(ret);
        return ret;
    }

    m_status.bytesWritten += len;
    return ESP_OK;
}

esp_err_t OTAService::writeSpiffsLocked(const uint8_t* data, size_t len) {
    if (m_spiffsPartition == nullptr || m_writeOffset + len > m_spiffsPartition->size) {
        m_status.message = "SPIFFS write exceeds storage partition";
        return ESP_ERR_INVALID_SIZE;
    }

    const esp_err_t ret = esp_partition_write(m_spiffsPartition, m_writeOffset, data, len);
    if (ret != ESP_OK) {
        m_status.message = std::string("SPIFFS write failed: ") + esp_err_to_name(ret);
        return ret;
    }

    m_writeOffset += len;
    m_status.bytesWritten += len;
    return ESP_OK;
}

esp_err_t OTAService::finish() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_activeTarget == OTAUpdateTarget::SPIFFS ? finishSpiffsLocked() : finishAppLocked();
}

esp_err_t OTAService::finishAppLocked() {
    if (!m_status.updateInProgress || m_updateHandle == 0 || m_updatePartition == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = esp_ota_end(m_updateHandle);
    m_updateHandle = 0;
    m_status.updateInProgress = false;
    if (ret != ESP_OK) {
        m_status.message = std::string("OTA validation failed: ") + esp_err_to_name(ret);
        return ret;
    }

    ret = esp_ota_set_boot_partition(m_updatePartition);
    if (ret != ESP_OK) {
        m_status.message = std::string("OTA boot partition update failed: ") + esp_err_to_name(ret);
        return ret;
    }

    m_status.rebootRequired = true;
    m_status.activeTarget = "app";
    m_status.message = "App OTA upload complete; reboot required";
    return ESP_OK;
}

esp_err_t OTAService::finishSpiffsLocked() {
    if (!m_status.updateInProgress || m_spiffsPartition == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if (m_status.expectedSize > 0 && m_status.bytesWritten != m_status.expectedSize) {
        m_status.updateInProgress = false;
        m_status.rebootRequired = true;
        m_status.message = "SPIFFS upload incomplete; serial reflash may be required";
        return ESP_ERR_INVALID_SIZE;
    }

    m_status.updateInProgress = false;
    m_status.rebootRequired = true;
    m_status.activeTarget = "spiffs";
    m_status.message = "SPIFFS OTA upload complete; reboot required";
    return ESP_OK;
}

void OTAService::abort() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_activeTarget == OTAUpdateTarget::APP && m_status.updateInProgress && m_updateHandle != 0) {
        (void)esp_ota_abort(m_updateHandle);
    }
    m_updateHandle = 0;
    m_status.updateInProgress = false;
    if (m_activeTarget == OTAUpdateTarget::SPIFFS) {
        m_status.rebootRequired = true;
        m_status.message = "SPIFFS upload aborted; serial reflash may be required";
    } else {
        m_status.message = "OTA upload aborted";
    }
    m_status.activeTarget = "none";
}
