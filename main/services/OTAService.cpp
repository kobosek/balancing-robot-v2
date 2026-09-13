#include "OTAService.hpp"
#include "IMUService.hpp"

#include "BaseEvent.hpp"
#include "OTA_UpdatePolicyChanged.hpp"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_random.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <cstdio>
#include <cerrno>
#include <algorithm>
#include <cstring>
#include <limits>

namespace {
constexpr const char* CONFIG_BACKUP_NAMESPACE = "ota_cfg";
constexpr const char* CONFIG_BACKUP_PENDING_KEY = "pending";
constexpr size_t CONFIG_BACKUP_CHUNK_SIZE = 1024;
constexpr size_t CONFIG_BACKUP_MAX_BYTES = 16384;

constexpr const char* BUNDLE_STATE_NAMESPACE = "ota_state";
constexpr const char* BUNDLE_STATE_STAGE_KEY = "stage";
constexpr const char* BUNDLE_STATE_ID_KEY = "bundle_id";
constexpr const char* BUNDLE_STATE_SPIFFS_SIZE_KEY = "spiffs_size";
constexpr const char* BUNDLE_STATE_APP_SIZE_KEY = "app_size";
constexpr const char* BUNDLE_STATE_APP_LABEL_KEY = "app_label";

std::string labelOrUnknown(const esp_partition_t* partition) {
    return partition ? std::string(partition->label) : std::string("unknown");
}

esp_err_t readFileForBackup(const char* path, std::string& output)
{
    output.clear();
    FILE* file = std::fopen(path, "rb");
    if (!file) return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
    if (std::fseek(file, 0, SEEK_END) != 0) {
        std::fclose(file);
        return ESP_FAIL;
    }
    const long length = std::ftell(file);
    if (length < 0 || static_cast<size_t>(length) > CONFIG_BACKUP_MAX_BYTES) {
        std::fclose(file);
        return ESP_ERR_INVALID_SIZE;
    }
    std::rewind(file);
    if (length > 0) {
        output.resize(static_cast<size_t>(length));
        const size_t read = std::fread(&output[0], 1, output.size(), file);
        if (read != output.size()) {
            output.clear();
            std::fclose(file);
            return ESP_FAIL;
        }
    }
    std::fclose(file);
    return ESP_OK;
}

esp_err_t ensureNvs()
{
    const esp_err_t ret = nvs_flash_init();
    if (ret == ESP_OK || ret == ESP_ERR_NVS_INVALID_STATE) return ESP_OK;
    return ret;
}

const char* bundleStageToString(OTABundleStage stage)
{
    switch (stage) {
        case OTABundleStage::SPIFFS_WRITING:    return "spiffs_writing";
        case OTABundleStage::SPIFFS_READY:      return "spiffs_ready";
        case OTABundleStage::APP_WRITING:       return "app_writing";
        case OTABundleStage::APP_PENDING_REBOOT:return "app_pending_reboot";
        case OTABundleStage::RECOVERY:          return "recovery";
        case OTABundleStage::NONE:
        default:                                return "none";
    }
}

bool isKnownBundleStage(uint8_t raw)
{
    return raw <= static_cast<uint8_t>(OTABundleStage::RECOVERY);
}
}

void OTAService::setBundleStageLocked(OTABundleStage stage,
                                       uint64_t bundleId,
                                       size_t spiffsSize,
                                       size_t appSize,
                                       const char* appLabel)
{
    m_bundleStage = stage;
    m_bundleId = bundleId;
    m_bundleSpiffsSize = spiffsSize;
    m_bundleAppSize = appSize;
    m_bundleAppLabel = appLabel ? appLabel : "";
    m_status.bundleId = m_bundleId;
    m_status.bundleStage = bundleStageToString(m_bundleStage);
}

void OTAService::setBundleRecoveryPendingLocked(bool pending)
{
    m_bundleRecoveryPending = pending;
    m_status.bundleRecoveryPending = pending;
    if (m_operationGate) {
        m_operationGate->setOtaRecoveryPending(pending);
    }
}

esp_err_t OTAService::persistBundleStateLocked(OTABundleStage stage,
                                               uint64_t bundleId,
                                               size_t spiffsSize,
                                               size_t appSize,
                                               const char* appLabel)
{
    if (stage != OTABundleStage::NONE &&
        (bundleId == 0 || appLabel == nullptr || appLabel[0] == '\0' ||
         spiffsSize == 0 || spiffsSize > std::numeric_limits<uint32_t>::max() ||
         appSize > std::numeric_limits<uint32_t>::max())) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t nvsRet = ensureNvs();
    if (nvsRet != ESP_OK) return nvsRet;

    nvs_handle_t handle = 0;
    esp_err_t ret = nvs_open(BUNDLE_STATE_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    if (stage == OTABundleStage::NONE) {
        ret = nvs_erase_all(handle);
        if (ret == ESP_OK) ret = nvs_commit(handle);
        nvs_close(handle);
        if (ret == ESP_OK) {
            setBundleStageLocked(OTABundleStage::NONE, 0, 0, 0, nullptr);
            setBundleRecoveryPendingLocked(false);
        }
        return ret;
    }

    // NVS commits the complete marker as one transaction.  The stage is
    // written together with its identity and expected lengths, so a torn
    // record is rejected by loadBundleStateLocked instead of being resumed.
    ret = nvs_set_u8(handle, BUNDLE_STATE_STAGE_KEY,
                     static_cast<uint8_t>(stage));
    if (ret == ESP_OK) {
        ret = nvs_set_u64(handle, BUNDLE_STATE_ID_KEY, bundleId);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u32(handle, BUNDLE_STATE_SPIFFS_SIZE_KEY,
                          static_cast<uint32_t>(spiffsSize));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u32(handle, BUNDLE_STATE_APP_SIZE_KEY,
                          static_cast<uint32_t>(appSize));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_str(handle, BUNDLE_STATE_APP_LABEL_KEY, appLabel);
    }
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    if (ret != ESP_OK) return ret;

    setBundleStageLocked(stage, bundleId, spiffsSize, appSize, appLabel);
    setBundleRecoveryPendingLocked(true);
    return ESP_OK;
}

esp_err_t OTAService::clearBundleStateLocked()
{
    const esp_err_t nvsRet = ensureNvs();
    if (nvsRet != ESP_OK) return nvsRet;

    nvs_handle_t handle = 0;
    esp_err_t ret = nvs_open(BUNDLE_STATE_NAMESPACE, NVS_READWRITE, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        setBundleStageLocked(OTABundleStage::NONE, 0, 0, 0, nullptr);
        setBundleRecoveryPendingLocked(false);
        return ESP_OK;
    }
    if (ret != ESP_OK) return ret;

    ret = nvs_erase_all(handle);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    if (ret != ESP_OK) return ret;

    setBundleStageLocked(OTABundleStage::NONE, 0, 0, 0, nullptr);
    setBundleRecoveryPendingLocked(false);
    return ESP_OK;
}

esp_err_t OTAService::loadBundleStateLocked()
{
    // A fresh process starts with no in-memory recovery latch.  Any durable
    // marker found below re-arms it before StateManager can enable motion.
    setBundleStageLocked(OTABundleStage::NONE, 0, 0, 0, nullptr);
    setBundleRecoveryPendingLocked(false);

    const esp_err_t nvsRet = ensureNvs();
    if (nvsRet != ESP_OK) {
        setBundleStageLocked(OTABundleStage::RECOVERY, 0, 0, 0, nullptr);
        setBundleRecoveryPendingLocked(true);
        m_status.message = std::string("OTA state storage unavailable: ") +
            esp_err_to_name(nvsRet);
        return ESP_OK;
    }

    nvs_handle_t handle = 0;
    esp_err_t ret = nvs_open(BUNDLE_STATE_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (ret != ESP_OK) {
        setBundleStageLocked(OTABundleStage::RECOVERY, 0, 0, 0, nullptr);
        setBundleRecoveryPendingLocked(true);
        m_status.message = std::string("OTA state could not be opened: ") +
            esp_err_to_name(ret);
        return ESP_OK;
    }

    uint8_t rawStage = 0;
    ret = nvs_get_u8(handle, BUNDLE_STATE_STAGE_KEY, &rawStage);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        // An empty namespace is harmless after a successful clear.  If any
        // identity field survived without the stage key, however, treat the
        // record as torn rather than silently allowing motion.
        uint64_t orphanedId = 0;
        const esp_err_t orphanRet = nvs_get_u64(handle, BUNDLE_STATE_ID_KEY,
                                                 &orphanedId);
        uint32_t orphanedSize = 0;
        const esp_err_t orphanSizeRet = nvs_get_u32(
            handle, BUNDLE_STATE_SPIFFS_SIZE_KEY, &orphanedSize);
        size_t orphanedLabelLength = 0;
        const esp_err_t orphanLabelRet = nvs_get_str(
            handle, BUNDLE_STATE_APP_LABEL_KEY, nullptr, &orphanedLabelLength);
        nvs_close(handle);
        if ((orphanRet == ESP_OK && orphanedId != 0) ||
            orphanSizeRet == ESP_OK || orphanLabelRet == ESP_OK) {
            setBundleStageLocked(OTABundleStage::RECOVERY, orphanedId,
                                 orphanedSize, 0, nullptr);
            setBundleRecoveryPendingLocked(true);
            m_status.message = "OTA state marker is torn; explicit recovery required";
        }
        return ESP_OK;
    }
    if (ret != ESP_OK || !isKnownBundleStage(rawStage)) {
        nvs_close(handle);
        setBundleStageLocked(OTABundleStage::RECOVERY, 0, 0, 0, nullptr);
        setBundleRecoveryPendingLocked(true);
        m_status.message = "OTA state marker is invalid; explicit recovery required";
        return ESP_OK;
    }

    const OTABundleStage stage = static_cast<OTABundleStage>(rawStage);
    if (stage == OTABundleStage::NONE) {
        nvs_close(handle);
        return ESP_OK;
    }

    uint64_t bundleId = 0;
    uint32_t spiffsSize = 0;
    uint32_t appSize = 0;
    char appLabel[17] = {}; // ESP-IDF partition labels are 16 chars + NUL.
    size_t appLabelLength = sizeof(appLabel);
    ret = nvs_get_u64(handle, BUNDLE_STATE_ID_KEY, &bundleId);
    if (ret == ESP_OK) ret = nvs_get_u32(handle, BUNDLE_STATE_SPIFFS_SIZE_KEY, &spiffsSize);
    if (ret == ESP_OK) ret = nvs_get_u32(handle, BUNDLE_STATE_APP_SIZE_KEY, &appSize);
    if (ret == ESP_OK) ret = nvs_get_str(handle, BUNDLE_STATE_APP_LABEL_KEY,
                                          appLabel, &appLabelLength);
    appLabel[sizeof(appLabel) - 1] = '\0';
    nvs_close(handle);

    const bool valid = ret == ESP_OK && bundleId != 0 && spiffsSize > 0 &&
        appLabel[0] != '\0' &&
        (m_spiffsPartition == nullptr || spiffsSize <= m_spiffsPartition->size) &&
        ((stage == OTABundleStage::APP_WRITING ||
          stage == OTABundleStage::APP_PENDING_REBOOT) ? appSize > 0 : true);
    if (!valid) {
        setBundleStageLocked(OTABundleStage::RECOVERY, bundleId, spiffsSize,
                             appSize, appLabel);
        setBundleRecoveryPendingLocked(true);
        m_status.message = "OTA state marker is incomplete; explicit recovery required";
        return ESP_OK;
    }

    setBundleStageLocked(stage, bundleId, spiffsSize, appSize, appLabel);
    setBundleRecoveryPendingLocked(true);
    if (stage == OTABundleStage::SPIFFS_READY) {
        // The exact SPIFFS byte count was committed before the marker.  It is
        // therefore safe to offer the second half after a reboot.
        m_bundleSpiffsReady = true;
        m_status.message = "SPIFFS bundle is ready; firmware upload required";
    } else if (stage == OTABundleStage::APP_PENDING_REBOOT &&
               m_runningPartition != nullptr &&
               std::string(m_runningPartition->label) == m_bundleAppLabel) {
        // The new app is running.  Clearing the marker is the durable proof
        // that the bundle crossed the reboot boundary successfully.
        esp_ota_img_states_t runningState;
        const esp_err_t stateRet = esp_ota_get_state_partition(
            m_runningPartition, &runningState);
        if (stateRet != ESP_OK ||
            runningState == ESP_OTA_IMG_PENDING_VERIFY ||
            runningState == ESP_OTA_IMG_INVALID ||
            runningState == ESP_OTA_IMG_ABORTED) {
            m_status.message = stateRet == ESP_OK
                ? "OTA bundle booted but image validation is still pending"
                : std::string("OTA bundle boot state unavailable: ") +
                    esp_err_to_name(stateRet);
            return ESP_OK;
        }
        const esp_err_t clearRet = clearBundleStateLocked();
        if (clearRet == ESP_OK) {
            m_status.message = "OTA bundle applied successfully";
        } else {
            m_status.message = std::string("OTA bundle applied; state cleanup pending: ") +
                esp_err_to_name(clearRet);
        }
    } else {
        m_status.message = std::string("OTA bundle recovery required at stage ") +
            bundleStageToString(stage);
    }
    return ESP_OK;
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
    m_status.updateAllowed = false;
    m_status.updateInProgress = false;
    m_status.rebootRequired = false;
    m_status.bytesWritten = 0;
    m_status.expectedSize = 0;
    m_status.spiffsPartitionSize = m_spiffsPartition ? m_spiffsPartition->size : 0;
    m_status.activeTarget = "none";
    m_status.bundleId = 0;
    m_status.bundleStage = "none";
    m_status.bundleRecoveryPending = false;
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

    // Restore the durable bundle marker before the runtime can accept any
    // motion.  A malformed/unreadable marker is fail-closed in the operation
    // gate but does not prevent the web endpoint from explaining recovery.
    // This runs after pending-image validation so a failed rollback check
    // cannot be mistaken for a successfully booted bundle.
    (void)loadBundleStateLocked();

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

ControlOperationStatus OTAService::getOperationStatus() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_operationGate) {
        return {};
    }
    const ControlOperationStatus current = m_operationGate->currentStatus();
    if (current.known && current.kind == ControlOperationKind::OTA) {
        return current;
    }
    const ControlOperationStatus last = m_operationGate->lastStatus();
    return last.known && last.kind == ControlOperationKind::OTA
        ? last : ControlOperationStatus{};
}

void OTAService::handleEvent(const BaseEvent& event) {
    if (event.is<OTA_UpdatePolicyChanged>()) {
        setUpdateAllowed(event.as<OTA_UpdatePolicyChanged>().updateAllowed);
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

void OTAService::setUpdateAllowed(bool allowed) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_status.updateAllowed = allowed;
    if (!allowed && !m_status.updateInProgress &&
        m_bundleStage == OTABundleStage::NONE) {
        m_bundleSpiffsReady = false;
    }
}

esp_err_t OTAService::backupConfigurationLocked()
{
    if (m_configurationBackupStored) return ESP_OK;

    std::string config;
    std::string operation;
    const esp_err_t configRet = readFileForBackup(
        "/spiffs/config.json", config);
    if (configRet != ESP_OK && configRet != ESP_ERR_NOT_FOUND) {
        m_status.message = "Configuration backup read failed before SPIFFS erase";
        return configRet;
    }
    const esp_err_t operationRet = readFileForBackup(
        "/spiffs/config.json.operation", operation);
    if (operationRet != ESP_OK && operationRet != ESP_ERR_NOT_FOUND) {
        m_status.message = "Configuration operation backup read failed before SPIFFS erase";
        return operationRet;
    }

    const esp_err_t nvsRet = ensureNvs();
    if (nvsRet != ESP_OK) {
        m_status.message = std::string("Configuration backup storage unavailable: ") +
            esp_err_to_name(nvsRet);
        return nvsRet;
    }

    nvs_handle_t handle = 0;
    esp_err_t ret = nvs_open(CONFIG_BACKUP_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        m_status.message = std::string("Configuration backup open failed: ") +
            esp_err_to_name(ret);
        return ret;
    }
    auto close = [&]() { nvs_close(handle); };
    ret = nvs_erase_all(handle);
    if (ret == ESP_OK) ret = nvs_set_u8(handle, CONFIG_BACKUP_PENDING_KEY, 1);
    const auto writeBlob = [&](const char* prefix, const std::string& data) -> esp_err_t {
        if (data.size() > CONFIG_BACKUP_MAX_BYTES) return ESP_ERR_INVALID_SIZE;
        const uint32_t length = static_cast<uint32_t>(data.size());
        const uint32_t chunks = static_cast<uint32_t>(
            (data.size() + CONFIG_BACKUP_CHUNK_SIZE - 1) / CONFIG_BACKUP_CHUNK_SIZE);
        char lengthKey[16] = {};
        char countKey[16] = {};
        std::snprintf(lengthKey, sizeof(lengthKey), "%s_len", prefix);
        std::snprintf(countKey, sizeof(countKey), "%s_cnt", prefix);
        esp_err_t result = nvs_set_u32(handle, lengthKey, length);
        if (result == ESP_OK) result = nvs_set_u32(handle, countKey, chunks);
        for (uint32_t index = 0; result == ESP_OK && index < chunks; ++index) {
            char key[16] = {};
            std::snprintf(key, sizeof(key), "%s%u", prefix,
                          static_cast<unsigned>(index));
            const size_t offset = static_cast<size_t>(index) * CONFIG_BACKUP_CHUNK_SIZE;
            const size_t count = std::min(CONFIG_BACKUP_CHUNK_SIZE,
                                          data.size() - offset);
            result = nvs_set_blob(handle, key, data.data() + offset, count);
        }
        return result;
    };
    if (ret == ESP_OK) ret = writeBlob("cfg", config);
    if (ret == ESP_OK) ret = writeBlob("op", operation);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    close();
    if (ret != ESP_OK) {
        m_status.message = std::string("Configuration backup failed: ") +
            esp_err_to_name(ret);
        return ret;
    }
    m_configurationBackupStored = true;
    ESP_LOGI(TAG, "Configuration and operation journal backed up before SPIFFS OTA erase");
    return ESP_OK;
}

void OTAService::clearConfigurationBackupLocked()
{
    if (!m_configurationBackupStored) return;
    if (ensureNvs() != ESP_OK) return;
    nvs_handle_t handle = 0;
    if (nvs_open(CONFIG_BACKUP_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return;
    if (nvs_erase_all(handle) == ESP_OK && nvs_commit(handle) == ESP_OK) {
        m_configurationBackupStored = false;
        ESP_LOGI(TAG, "Configuration OTA backup cleared after app partition commit");
    }
    nvs_close(handle);
}

bool OTAService::tryReserveOtaLocked()
{
    if (m_operationReservation.valid()) {
        return true;
    }
    if (!m_operationGate) {
        return true;
    }
    if (m_operationGate->tryAcquire(ControlOperationKind::OTA,
                                    m_operationReservation)) {
        (void)m_operationGate->updatePhase(
            m_operationReservation, ControlOperationPhase::PREPARING);
        return true;
    }
    m_status.message = "Another control operation is in progress";
    return false;
}

void OTAService::releaseOperationReservationLocked(
    ControlOperationPhase finalPhase,
    int32_t resultCode)
{
    if (m_operationReservation.valid() && m_operationGate) {
        (void)m_operationGate->release(m_operationReservation,
                                        finalPhase,
                                        resultCode);
    }
}

esp_err_t OTAService::begin(size_t expectedSize) {
    return beginAppUpdate(expectedSize);
}

esp_err_t OTAService::beginAppUpdate(size_t expectedSize) {
    std::lock_guard<std::mutex> lock(m_mutex);
    const bool hadReservation = m_operationReservation.valid();
    if (!tryReserveOtaLocked()) return ESP_ERR_INVALID_STATE;
    if (!m_status.updateAllowed || !m_imu || !m_imu->reserveOta()) {
        if (!hadReservation) {
            releaseOperationReservationLocked(ControlOperationPhase::FAILED,
                                              ESP_ERR_INVALID_STATE);
        }
        return ESP_ERR_INVALID_STATE;
    }
    const auto result = beginAppUpdateLocked(expectedSize);
    if (result == ESP_OK && m_operationGate && m_operationReservation.valid()) {
        (void)m_operationGate->updatePhase(
            m_operationReservation, ControlOperationPhase::RUNNING);
    }
    if (result != ESP_OK && !m_status.updateInProgress) {
        // A failed app begin after a completed SPIFFS half must not leave the
        // shared gate or the IMU reservation held forever. The SPIFFS image
        // has already replaced the live filesystem, so force the bundle back
        // to an incomplete state and keep the recovery warning visible.
        if (m_bundleSpiffsReady) {
            m_bundleSpiffsReady = false;
            m_status.rebootRequired = true;
            m_status.message += "; SPIFFS bundle is incomplete; serial reflash may be required";
        }
        if (m_imu) m_imu->releaseOta();
        releaseOperationReservationLocked(ControlOperationPhase::FAILED, result);
    }
    return result;
}

esp_err_t OTAService::beginSpiffsUpdate(size_t expectedSize) {
    std::lock_guard<std::mutex> lock(m_mutex);
    const bool hadReservation = m_operationReservation.valid();
    if (!tryReserveOtaLocked()) return ESP_ERR_INVALID_STATE;
    if (!m_status.updateAllowed || !m_imu || !m_imu->reserveOta()) {
        if (!hadReservation) {
            releaseOperationReservationLocked(ControlOperationPhase::FAILED,
                                              ESP_ERR_INVALID_STATE);
        }
        return ESP_ERR_INVALID_STATE;
    }
    const auto result = beginSpiffsUpdateLocked(expectedSize);
    if (result == ESP_OK && m_operationGate && m_operationReservation.valid()) {
        (void)m_operationGate->updatePhase(
            m_operationReservation, ControlOperationPhase::RUNNING);
    }
    if (result != ESP_OK && !m_status.updateInProgress) {
        if (m_bundleSpiffsReady) {
            m_bundleSpiffsReady = false;
            m_status.rebootRequired = true;
            m_status.message += "; SPIFFS bundle is incomplete; serial reflash may be required";
        }
        if (m_imu) m_imu->releaseOta();
        releaseOperationReservationLocked(ControlOperationPhase::FAILED, result);
    }
    return result;
}

esp_err_t OTAService::beginAppUpdateLocked(size_t expectedSize) {
    if (!m_status.updateAllowed) {
        m_status.message = "OTA is allowed only while system is IDLE";
        return ESP_ERR_INVALID_STATE;
    }
    if (m_status.updateInProgress) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!m_bundleSpiffsReady) {
        m_status.message = "Upload SPIFFS image before firmware";
        return ESP_ERR_INVALID_STATE;
    }

    m_updatePartition = esp_ota_get_next_update_partition(nullptr);
    if (m_updatePartition == nullptr) {
        m_status.available = false;
        m_status.message = "No OTA update partition available";
        return ESP_ERR_NOT_FOUND;
    }
    if (m_bundleAppLabel == "unknown") {
        m_bundleAppLabel = std::string(m_updatePartition->label);
    } else if (!m_bundleAppLabel.empty() &&
               std::string(m_updatePartition->label) != m_bundleAppLabel) {
        m_status.message = "OTA bundle target partition changed; upload a new SPIFFS image";
        setBundleRecoveryPendingLocked(true);
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t ret = esp_ota_begin(
        m_updatePartition,
        expectedSize > 0 ? expectedSize : OTA_SIZE_UNKNOWN,
        &m_updateHandle);
    if (ret != ESP_OK) {
        m_status.message = std::string("OTA begin failed: ") + esp_err_to_name(ret);
        return ret;
    }

    const esp_err_t markerRet = persistBundleStateLocked(
        OTABundleStage::APP_WRITING,
        m_bundleId,
        m_bundleSpiffsSize,
        expectedSize,
        m_bundleAppLabel.c_str());
    if (markerRet != ESP_OK) {
        (void)esp_ota_abort(m_updateHandle);
        m_updateHandle = 0;
        m_status.message = std::string("OTA bundle marker could not be updated: ") +
            esp_err_to_name(markerRet);
        setBundleRecoveryPendingLocked(true);
        return markerRet;
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
    if (!m_status.updateAllowed) {
        m_status.message = "OTA is allowed only while system is IDLE";
        return ESP_ERR_INVALID_STATE;
    }
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

    // The SPIFFS image replaces the partition wholesale. Persist the current
    // configuration and operation sidecar in NVS before unmounting; a failed
    // backup leaves the live filesystem untouched and aborts the OTA.
    esp_err_t ret = backupConfigurationLocked();
    if (ret != ESP_OK) {
        return ret;
    }

    // Record the bundle before unmounting or erasing SPIFFS.  The marker is
    // intentionally durable even when the following destructive step fails.
    m_updatePartition = esp_ota_get_next_update_partition(nullptr);
    uint64_t bundleId = (static_cast<uint64_t>(esp_random()) << 32) |
        static_cast<uint64_t>(esp_random());
    if (bundleId == 0) bundleId = 1;
    const std::string appLabel = labelOrUnknown(m_updatePartition);
    ret = persistBundleStateLocked(OTABundleStage::SPIFFS_WRITING,
                                   bundleId,
                                   expectedSize,
                                   0,
                                   appLabel.c_str());
    if (ret != ESP_OK) {
        m_status.message = std::string("OTA bundle marker could not be stored: ") +
            esp_err_to_name(ret);
        setBundleRecoveryPendingLocked(true);
        return ret;
    }

    ret = esp_vfs_spiffs_unregister(SPIFFS_PARTITION_LABEL);
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
    m_bundleSpiffsReady = false;
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

    if (m_operationGate && m_operationReservation.valid()) {
        (void)m_operationGate->updatePhase(
            m_operationReservation, ControlOperationPhase::APPLYING);
    }
    if (m_status.expectedSize > 0 &&
        m_status.bytesWritten != m_status.expectedSize) {
        (void)esp_ota_abort(m_updateHandle);
        m_updateHandle = 0;
        m_status.updateInProgress = false;
        m_status.rebootRequired = false;
        m_status.message = "App upload incomplete; serial reflash may be required";
        setBundleRecoveryPendingLocked(true);
        if (m_imu) m_imu->releaseOta();
        releaseOperationReservationLocked(ControlOperationPhase::FAILED,
                                           ESP_ERR_INVALID_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t ret = esp_ota_end(m_updateHandle);
    m_updateHandle = 0;
    m_status.updateInProgress = false;
    if (ret != ESP_OK) {
        m_status.message = std::string("OTA validation failed: ") + esp_err_to_name(ret);
        if (m_imu) m_imu->releaseOta();
        releaseOperationReservationLocked(ControlOperationPhase::FAILED, ret);
        return ret;
    }

    ret = esp_ota_set_boot_partition(m_updatePartition);
    if (ret != ESP_OK) {
        m_status.message = std::string("OTA boot partition update failed: ") + esp_err_to_name(ret);
        if (m_imu) m_imu->releaseOta();
        releaseOperationReservationLocked(ControlOperationPhase::FAILED, ret);
        return ret;
    }

    const esp_err_t markerRet = persistBundleStateLocked(
        OTABundleStage::APP_PENDING_REBOOT,
        m_bundleId,
        m_bundleSpiffsSize,
        m_bundleAppSize,
        m_bundleAppLabel.c_str());
    if (markerRet != ESP_OK) {
        // The app partition and boot selector are already committed, so keep
        // the RAM recovery latch and the configuration copy.  The caller is
        // told to restart/recover explicitly instead of observing a false
        // successful bundle state.
        m_status.rebootRequired = true;
        m_bundleSpiffsReady = false;
        m_status.activeTarget = "app";
        m_status.message = std::string("App committed but OTA state marker failed: ") +
            esp_err_to_name(markerRet);
        setBundleRecoveryPendingLocked(true);
        if (m_imu) m_imu->releaseOta();
        releaseOperationReservationLocked(ControlOperationPhase::FAILED, markerRet);
        return markerRet;
    }

    m_status.rebootRequired = true;
    m_bundleSpiffsReady = false;
    m_status.activeTarget = "app";
    m_status.message = "App OTA upload complete; reboot required";
    // The NVS copy is only needed until both halves of the bundle have been
    // committed.  Keep it when cleanup fails so a reset can still restore the
    // last known configuration, but expose the condition to the operator.
    if (m_configurationBackupStored) {
        clearConfigurationBackupLocked();
        if (m_configurationBackupStored) {
            m_status.message += "; configuration backup cleanup pending";
        }
    }
    releaseOperationReservationLocked(ControlOperationPhase::SUCCEEDED, ESP_OK);
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
        setBundleRecoveryPendingLocked(true);
        if (m_imu) m_imu->releaseOta();
        releaseOperationReservationLocked(ControlOperationPhase::FAILED,
                                           ESP_ERR_INVALID_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    const esp_err_t markerRet = persistBundleStateLocked(
        OTABundleStage::SPIFFS_READY,
        m_bundleId,
        m_bundleSpiffsSize,
        m_bundleAppSize,
        m_bundleAppLabel.c_str());
    if (markerRet != ESP_OK) {
        m_status.updateInProgress = false;
        m_status.rebootRequired = true;
        m_status.message = std::string("SPIFFS complete but bundle marker failed: ") +
            esp_err_to_name(markerRet);
        setBundleRecoveryPendingLocked(true);
        if (m_imu) m_imu->releaseOta();
        releaseOperationReservationLocked(ControlOperationPhase::FAILED, markerRet);
        return markerRet;
    }

    m_status.updateInProgress = false;
    m_status.rebootRequired = false;
    m_bundleSpiffsReady = true;
    m_status.activeTarget = "spiffs";
    m_status.message = "SPIFFS OTA upload complete; firmware upload required";
    // SPIFFS is the first half of a bundled update. Keep the reservation
    // active so a second writer cannot interleave before the app image.
    if (m_operationGate && m_operationReservation.valid()) {
        (void)m_operationGate->updatePhase(
            m_operationReservation, ControlOperationPhase::RUNNING);
    }
    return ESP_OK;
}

void OTAService::abort() {
    std::lock_guard<std::mutex> lock(m_mutex);

    const bool hadBundleState = m_status.updateInProgress ||
        m_bundleStage != OTABundleStage::NONE;
    if (m_activeTarget == OTAUpdateTarget::APP && m_status.updateInProgress && m_updateHandle != 0) {
        (void)esp_ota_abort(m_updateHandle);
    }
    m_updateHandle = 0;
    m_status.updateInProgress = false;
    if (m_activeTarget == OTAUpdateTarget::SPIFFS) {
        m_bundleSpiffsReady = false;
        m_status.rebootRequired = true;
        m_status.message = "SPIFFS upload aborted; serial reflash may be required";
    } else {
        m_status.message = "OTA upload aborted";
    }
    m_status.activeTarget = "none";
    if (hadBundleState) {
        const std::string appLabel = m_bundleAppLabel.empty()
            ? labelOrUnknown(m_updatePartition) : m_bundleAppLabel;
        uint64_t bundleId = m_bundleId;
        if (bundleId == 0) {
            bundleId = (static_cast<uint64_t>(esp_random()) << 32) |
                static_cast<uint64_t>(esp_random());
            if (bundleId == 0) bundleId = 1;
        }
        const size_t spiffsSize = m_bundleSpiffsSize != 0
            ? m_bundleSpiffsSize
            : (m_spiffsPartition ? m_spiffsPartition->size : 1);
        const esp_err_t markerRet = persistBundleStateLocked(
            OTABundleStage::RECOVERY,
            bundleId,
            spiffsSize,
            m_bundleAppSize,
            appLabel.c_str());
        if (markerRet != ESP_OK) {
            setBundleStageLocked(OTABundleStage::RECOVERY, bundleId,
                                 spiffsSize, m_bundleAppSize, appLabel.c_str());
            setBundleRecoveryPendingLocked(true);
            m_status.message += "; OTA recovery marker could not be stored";
        }
    }
    if (m_imu) m_imu->releaseOta();
    releaseOperationReservationLocked(ControlOperationPhase::ABORTED,
                                       ESP_ERR_INVALID_STATE);
}
