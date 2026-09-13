#include "ConfigurationService.hpp"     // Relative path within module's include dir
#include "SPIFFSStorageService.hpp"     // Relative path within module's include dir
#include "JsonConfigParser.hpp"         // Relative path within module's include dir
#include "EventBus.hpp"                 // Found via INCLUDE_DIRS
#include "ConfigData.hpp"               // Found via INCLUDE_DIRS (needed for constructor/defaults)
#include "BaseEvent.hpp"                // Found via INCLUDE_DIRS (needed for publish)
#include "IMU_GyroOffsetsUpdated.hpp" // For handling gyro offset updates

#include <string>
#include <inttypes.h>
#include <limits>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <functional>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>
#include <utility>
#include "cJSON.h"
#include "esp_log.h"                    // Moved from header

namespace {

bool parseUnsignedDecimal(cJSON* item, uint64_t& value, bool allowZero)
{
    if (!item) {
        value = 0;
        return false;
    }

    if (cJSON_IsString(item) && item->valuestring) {
        const char* text = item->valuestring;
        if (*text == '\0' || *text == '-') {
            return false;
        }
        for (const char* cursor = text; *cursor != '\0'; ++cursor) {
            if (*cursor < '0' || *cursor > '9') return false;
        }
        errno = 0;
        char* end = nullptr;
        const unsigned long long parsed = std::strtoull(text, &end, 10);
        if (errno != 0 || end == text || *end != '\0' ||
            (!allowZero && parsed == 0)) {
            return false;
        }
        value = static_cast<uint64_t>(parsed);
        return allowZero || value != 0;
    }

    // JSON numbers cannot represent every uint64 value exactly.  Accept only
    // the exact integer range that a browser can round-trip; clients needing
    // the full range use the decimal string form above.
    if (cJSON_IsNumber(item) && std::isfinite(item->valuedouble) &&
        item->valuedouble >= (allowZero ? 0.0 : 1.0) &&
        item->valuedouble <= 9007199254740991.0 &&
        std::floor(item->valuedouble) == item->valuedouble) {
        value = static_cast<uint64_t>(item->valuedouble);
        return allowZero || value != 0;
    }
    return false;
}

bool parseOperationId(cJSON* item, uint64_t& operationId)
{
    return parseUnsignedDecimal(item, operationId, false);
}

const char* operationPhaseToString(ControlOperationPhase phase)
{
    switch (phase) {
        case ControlOperationPhase::RESERVED:  return "reserved";
        case ControlOperationPhase::PREPARING: return "preparing";
        case ControlOperationPhase::RUNNING:   return "running";
        case ControlOperationPhase::WRITING:   return "writing";
        case ControlOperationPhase::APPLYING:  return "applying";
        case ControlOperationPhase::SUCCEEDED: return "succeeded";
        case ControlOperationPhase::FAILED:    return "failed";
        case ControlOperationPhase::ABORTED:   return "aborted";
        case ControlOperationPhase::IDLE:
        default:                                return "idle";
    }
}

bool operationPhaseFromString(const char* text, ControlOperationPhase& phase)
{
    if (!text) return false;
    const struct Entry { const char* name; ControlOperationPhase phase; } entries[] = {
        {"reserved", ControlOperationPhase::RESERVED},
        {"preparing", ControlOperationPhase::PREPARING},
        {"running", ControlOperationPhase::RUNNING},
        {"writing", ControlOperationPhase::WRITING},
        {"applying", ControlOperationPhase::APPLYING},
        {"succeeded", ControlOperationPhase::SUCCEEDED},
        {"failed", ControlOperationPhase::FAILED},
        {"aborted", ControlOperationPhase::ABORTED},
        {"idle", ControlOperationPhase::IDLE}
    };
    for (const auto& entry : entries) {
        if (std::strcmp(text, entry.name) == 0) {
            phase = entry.phase;
            return true;
        }
    }
    return false;
}

uint64_t stableHash(const std::string& value)
{
    // FNV-1a keeps the operation fingerprint deterministic across reboots and
    // toolchains; std::hash is not required to be stable by the C++ standard.
    uint64_t hash = 1469598103934665603ULL;
    for (const unsigned char byte : value) {
        hash ^= byte;
        hash *= 1099511628211ULL;
    }
    return hash;
}

void appendCanonicalToken(std::string& output,
                          char type,
                          const std::string& value)
{
    output.push_back(type);
    output += std::to_string(value.size());
    output.push_back(':');
    output += value;
    output.push_back(';');
}

bool appendCanonicalJson(const cJSON* item, std::string& output)
{
    if (!item) return false;
    if (cJSON_IsNull(item)) {
        appendCanonicalToken(output, 'n', "");
        return true;
    }
    if (cJSON_IsTrue(item)) {
        appendCanonicalToken(output, 'b', "1");
        return true;
    }
    if (cJSON_IsFalse(item)) {
        appendCanonicalToken(output, 'b', "0");
        return true;
    }
    if (cJSON_IsNumber(item)) {
        char number[48] = {};
        const int length = std::snprintf(number, sizeof(number), "%.17g",
                                         item->valuedouble);
        if (length <= 0 || static_cast<size_t>(length) >= sizeof(number)) {
            return false;
        }
        appendCanonicalToken(output, 'd', std::string(number, length));
        return true;
    }
    if (cJSON_IsString(item) && item->valuestring) {
        appendCanonicalToken(output, 's', item->valuestring);
        return true;
    }
    if (cJSON_IsArray(item)) {
        output += "a[";
        for (const cJSON* child = item->child; child; child = child->next) {
            if (!appendCanonicalJson(child, output)) return false;
        }
        output += "];";
        return true;
    }
    if (cJSON_IsObject(item)) {
        std::vector<std::pair<std::string, const cJSON*>> children;
        for (const cJSON* child = item->child; child; child = child->next) {
            children.emplace_back(child->string ? child->string : "", child);
        }
        std::sort(children.begin(), children.end(),
                  [](const auto& left, const auto& right) {
                      return left.first < right.first;
                  });
        output += "o{";
        for (const auto& entry : children) {
            appendCanonicalToken(output, 'k', entry.first);
            if (!appendCanonicalJson(entry.second, output)) return false;
        }
        output += "};";
        return true;
    }
    return false;
}

uint64_t requestFingerprint(const std::string& json)
{
    // The operation identifier is transport metadata.  Excluding it from
    // the fingerprint lets a client retry the same document with the same
    // operation_id even when a JSON serializer changes whitespace or key
    // ordering. If parsing fails, retain a deterministic raw fallback; the
    // parser below will return the actual format error.
    if (cJSON* root = cJSON_Parse(json.c_str())) {
        cJSON_DeleteItemFromObjectCaseSensitive(root, "operation_id");
        cJSON_DeleteItemFromObjectCaseSensitive(root, "operationId");
        std::string canonical;
        if (appendCanonicalJson(root, canonical)) {
            const uint64_t fingerprint = stableHash(canonical);
            cJSON_Delete(root);
            return fingerprint;
        }
        cJSON_Delete(root);
    }
    return stableHash(json);
}

} // namespace

ConfigurationService::ConfigurationService(IStorageService& storage,
                                           IConfigParser& parser,
                                           EventBus& bus,
                                           const std::string& configKey,
                                           ControlOperationGate* operationGate) :
    m_storageService(storage),
    m_configParser(parser),
    m_eventBus(bus),
    m_configKey(configKey),
    m_operationJournalKey(configKey + ".operation"),
    m_operationGate(operationGate),
    m_configChangePublisher(bus)
{}

esp_err_t ConfigurationService::persistOperationJournal(
    uint64_t operationId,
    uint64_t fingerprint,
    uint32_t baseRevision,
    uint32_t targetRevision,
    ControlOperationPhase phase,
    int32_t resultCode)
{
    if (operationId == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;
    cJSON_AddNumberToObject(root, "version", 1);

    char operationIdText[24] = {};
    char fingerprintText[24] = {};
    std::snprintf(operationIdText, sizeof(operationIdText), "%llu",
                  static_cast<unsigned long long>(operationId));
    std::snprintf(fingerprintText, sizeof(fingerprintText), "%llu",
                  static_cast<unsigned long long>(fingerprint));
    cJSON_AddStringToObject(root, "operation_id", operationIdText);
    cJSON_AddStringToObject(root, "fingerprint", fingerprintText);
    cJSON_AddNumberToObject(root, "base_revision", baseRevision);
    cJSON_AddNumberToObject(root, "target_revision", targetRevision);
    cJSON_AddStringToObject(root, "phase", operationPhaseToString(phase));
    cJSON_AddNumberToObject(root, "result_code", resultCode);

    char* printed = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!printed) return ESP_ERR_NO_MEM;
    const std::string serialized(printed);
    cJSON_free(printed);

    esp_err_t ret = m_storageService.saveData(m_operationJournalKey, serialized);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to persist operation journal (%s)",
                 esp_err_to_name(ret));
        return ret;
    }
    std::string readback;
    ret = m_storageService.loadData(m_operationJournalKey, readback);
    if (ret != ESP_OK || readback != serialized) {
        ESP_LOGE(TAG, "Operation journal readback failed (%s)",
                 ret == ESP_OK ? "content-mismatch" : esp_err_to_name(ret));
        const esp_err_t restoreRet = m_storageService.restoreBackup(m_operationJournalKey);
        if (restoreRet == ESP_OK) {
            ESP_LOGW(TAG, "Operation journal rollback attempted after readback failure");
        }
        return ret == ESP_OK ? ESP_FAIL : ret;
    }
    // SPIFFS keeps the previous journal beside the newly published record
    // until this verification point.  A backend without that facility uses a
    // no-op default; a failed cleanup is still an uncertain transaction.
    ret = m_storageService.commitData(m_operationJournalKey);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Operation journal backup cleanup failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t ConfigurationService::loadOperationJournal()
{
    std::string raw;
    const esp_err_t loadRet = m_storageService.loadData(m_operationJournalKey, raw);
    if (loadRet == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }
    if (loadRet != ESP_OK) {
        // An unreadable sidecar is an unknown transaction state, not an
        // empty journal.  Keep the gate latched until an explicit full v3
        // repair is successfully persisted and read back.
        ESP_LOGW(TAG, "Operation journal unavailable (%s); keeping recovery latched",
                 esp_err_to_name(loadRet));
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        m_operationRecoveryPending = true;
        m_recoveryOperationId = 0;
        m_recoveryOperationFingerprint = 0;
        m_recoveryBaseRevision = 0;
        m_recoveryTargetRevision = 0;
        m_lastOperationResult = ESP_ERR_INVALID_STATE;
        m_lastOperationPhase = ControlOperationPhase::ABORTED;
        m_hasLastOperation = false;
        if (m_operationGate) m_operationGate->setRecoveryPending(true);
        return loadRet;
    }

    cJSON* root = cJSON_Parse(raw.c_str());
    if (!root || !cJSON_IsObject(root)) {
        if (root) cJSON_Delete(root);
        ESP_LOGE(TAG, "Operation journal is malformed; keeping motion inhibited");
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        m_operationRecoveryPending = true;
        m_recoveryOperationId = 0;
        m_recoveryOperationFingerprint = 0;
        m_recoveryBaseRevision = 0;
        m_recoveryTargetRevision = 0;
        m_hasLastOperation = false;
        if (m_operationGate) m_operationGate->setRecoveryPending(true);
        return ESP_FAIL;
    }

    uint64_t operationId = 0;
    uint64_t fingerprint = 0;
    cJSON* idItem = cJSON_GetObjectItem(root, "operation_id");
    cJSON* fingerprintItem = cJSON_GetObjectItem(root, "fingerprint");
    cJSON* versionItem = cJSON_GetObjectItem(root, "version");
    cJSON* phaseItem = cJSON_GetObjectItem(root, "phase");
    cJSON* baseItem = cJSON_GetObjectItem(root, "base_revision");
    cJSON* targetItem = cJSON_GetObjectItem(root, "target_revision");
    cJSON* resultItem = cJSON_GetObjectItem(root, "result_code");
    ControlOperationPhase phase = ControlOperationPhase::IDLE;
    const bool validId = parseUnsignedDecimal(idItem, operationId, false);
    const bool validFingerprint = parseUnsignedDecimal(
        fingerprintItem, fingerprint, true);
    const bool validVersion = versionItem && cJSON_IsNumber(versionItem) &&
        std::isfinite(versionItem->valuedouble) &&
        versionItem->valuedouble == 1.0;
    const bool validPhase = phaseItem && cJSON_IsString(phaseItem) &&
        operationPhaseFromString(phaseItem->valuestring, phase);
    const bool validBase = baseItem && cJSON_IsNumber(baseItem) &&
        std::isfinite(baseItem->valuedouble) && baseItem->valuedouble >= 0.0 &&
        baseItem->valuedouble <= 4294967295.0 &&
        std::floor(baseItem->valuedouble) == baseItem->valuedouble;
    const bool validTarget = targetItem && cJSON_IsNumber(targetItem) &&
        std::isfinite(targetItem->valuedouble) && targetItem->valuedouble >= 0.0 &&
        targetItem->valuedouble <= 4294967295.0 &&
        std::floor(targetItem->valuedouble) == targetItem->valuedouble;
    const bool validResult = resultItem && cJSON_IsNumber(resultItem) &&
        std::isfinite(resultItem->valuedouble) &&
        resultItem->valuedouble >= static_cast<double>(INT32_MIN) &&
        resultItem->valuedouble <= static_cast<double>(INT32_MAX) &&
        std::floor(resultItem->valuedouble) == resultItem->valuedouble;
    const double baseRevisionValue = validBase ? baseItem->valuedouble : 0.0;
    const double targetRevisionValue = validTarget ? targetItem->valuedouble : 0.0;
    const double resultCodeValue = validResult ? resultItem->valuedouble : 0.0;
    cJSON_Delete(root);

    if (!validVersion || !validId || !validFingerprint || !validPhase || !validBase ||
        !validTarget || !validResult) {
        ESP_LOGE(TAG, "Operation journal fields are invalid; keeping motion inhibited");
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        m_operationRecoveryPending = true;
        m_recoveryOperationId = 0;
        m_recoveryOperationFingerprint = 0;
        m_recoveryBaseRevision = 0;
        m_recoveryTargetRevision = 0;
        m_hasLastOperation = false;
        if (m_operationGate) m_operationGate->setRecoveryPending(true);
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t baseRevision = static_cast<uint32_t>(baseRevisionValue);
    const uint32_t targetRevision = static_cast<uint32_t>(targetRevisionValue);
    const int32_t resultCode = static_cast<int32_t>(resultCodeValue);
    const bool interrupted = phase == ControlOperationPhase::RESERVED ||
        phase == ControlOperationPhase::PREPARING ||
        phase == ControlOperationPhase::RUNNING ||
        phase == ControlOperationPhase::WRITING ||
        phase == ControlOperationPhase::APPLYING;

    std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
    m_lastOperationId = operationId;
    m_lastOperationFingerprint = fingerprint;
    m_lastOperationResult = interrupted ? ESP_ERR_INVALID_STATE : resultCode;
    m_lastOperationPhase = interrupted ? ControlOperationPhase::ABORTED : phase;
    m_lastOperationBaseRevision = baseRevision;
    m_lastOperationTargetRevision = targetRevision;
    m_hasLastOperation = true;
    m_operationRecoveryPending = interrupted;
    m_recoveryOperationId = interrupted ? operationId : 0;
    m_recoveryOperationFingerprint = interrupted ? fingerprint : 0;
    m_recoveryBaseRevision = interrupted ? baseRevision : 0;
    m_recoveryTargetRevision = interrupted ? targetRevision : 0;
    if (m_operationGate) m_operationGate->setRecoveryPending(interrupted);
    if (interrupted) {
        ESP_LOGW(TAG, "Recovered interrupted configuration operation %llu (phase=%s)",
                 static_cast<unsigned long long>(operationId),
                 operationPhaseToString(phase));
    }
    return ESP_OK;
}

uint64_t ConfigurationService::getLastOperationId() const
{
    std::lock_guard<std::mutex> lock(m_operationStatusMutex);
    return m_lastOperationId;
}

ControlOperationStatus ConfigurationService::getOperationStatus() const
{
    {
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        if (m_operationRecoveryPending || m_activeOperationId != 0) {
            ControlOperationStatus status;
            status.known = m_recoveryOperationId != 0 ||
                m_activeOperationId != 0 || m_hasLastOperation;
            status.active = m_activeOperationId != 0;
            status.kind = status.known ? ControlOperationKind::CONFIGURATION
                                        : ControlOperationKind::NONE;
            status.operationId = m_activeOperationId != 0
                ? m_activeOperationId
                : (m_recoveryOperationId != 0 ? m_recoveryOperationId
                                               : m_lastOperationId);
            status.phase = m_activeOperationId != 0
                ? m_activeOperationPhase
                : (m_operationRecoveryPending ? ControlOperationPhase::ABORTED
                                               : m_lastOperationPhase);
            status.resultCode = m_operationRecoveryPending
                ? ESP_ERR_INVALID_STATE : m_lastOperationResult;
            status.baseRevision = m_activeOperationId != 0
                ? m_activeOperationBaseRevision : m_recoveryBaseRevision;
            status.targetRevision = m_activeOperationId != 0
                ? m_activeOperationTargetRevision : m_recoveryTargetRevision;
            return status;
        }
        if (m_hasLastOperation) {
            ControlOperationStatus status;
            status.known = true;
            status.active = false;
            status.kind = ControlOperationKind::CONFIGURATION;
            status.operationId = m_lastOperationId;
            status.phase = m_lastOperationPhase;
            status.resultCode = m_lastOperationResult;
            status.baseRevision = m_lastOperationBaseRevision;
            status.targetRevision = m_lastOperationTargetRevision;
            return status;
        }
    }
    if (m_operationGate) {
        const ControlOperationStatus current = m_operationGate->currentStatus();
        if (current.known && current.kind == ControlOperationKind::CONFIGURATION) {
            return current;
        }
        const ControlOperationStatus last = m_operationGate->lastStatus();
        if (last.known && last.kind == ControlOperationKind::CONFIGURATION) {
            return last;
        }
    }

    std::lock_guard<std::mutex> lock(m_operationStatusMutex);
    ControlOperationStatus status;
    status.known = m_hasLastOperation || m_activeOperationId != 0;
    status.active = m_activeOperationId != 0;
    status.kind = status.known ? ControlOperationKind::CONFIGURATION
                               : ControlOperationKind::NONE;
    status.operationId = m_activeOperationId != 0
        ? m_activeOperationId : m_lastOperationId;
    status.phase = m_activeOperationId != 0
        ? m_activeOperationPhase : m_lastOperationPhase;
    status.resultCode = m_lastOperationResult;
    status.baseRevision = m_activeOperationId != 0
        ? m_activeOperationBaseRevision : m_lastOperationBaseRevision;
    status.targetRevision = m_activeOperationId != 0
        ? m_activeOperationTargetRevision : m_lastOperationTargetRevision;
    return status;
}

bool ConfigurationService::isOperationRecoveryPending() const
{
    std::lock_guard<std::mutex> lock(m_operationStatusMutex);
    return m_operationRecoveryPending;
}

esp_err_t ConfigurationService::acknowledgeOperation(uint64_t operationId)
{
    if (operationId == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ControlOperationReservation reservation;
    uint64_t fingerprint = 0;
    uint32_t baseRevision = 0;
    uint32_t targetRevision = 0;
    ConfigData currentConfig;
    bool replayPersistedDocument = false;
    {
        std::lock_guard<std::mutex> operationLock(m_operationMutex);
        {
            std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
            if (!m_operationRecoveryPending || m_recoveryOperationId != operationId) {
                return ESP_ERR_NOT_FOUND;
            }
            fingerprint = m_recoveryOperationFingerprint;
            baseRevision = m_recoveryBaseRevision;
            targetRevision = m_recoveryTargetRevision;
        }
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            currentConfig = m_configData;
        }
        const bool atTargetRevision = targetRevision != 0 &&
            currentConfig.config_revision == targetRevision;
        const bool atBaseRevision = currentConfig.config_revision == baseRevision;
        if (!atTargetRevision && !atBaseRevision) {
            ESP_LOGE(TAG, "Cannot reconcile operation %llu: config revision %" PRIu32
                     " does not match base %" PRIu32 " or target %" PRIu32,
                     static_cast<unsigned long long>(operationId),
                     currentConfig.config_revision, baseRevision, targetRevision);
            return ESP_ERR_INVALID_STATE;
        }
        // If the reset happened before the document was published, the
        // current file is still the base revision. Acknowledgement then
        // records a deterministic abort and clears the latch. If the target
        // revision is already present, replaying the full snapshot completes
        // the apply half without writing the document a second time.
        replayPersistedDocument = atTargetRevision && targetRevision != baseRevision;
        if (!tryReserveConfiguration(reservation, "operation recovery", operationId)) {
            return ESP_ERR_INVALID_STATE;
        }
        {
            std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
            m_activeOperationId = operationId;
            m_activeOperationPhase = replayPersistedDocument
                ? ControlOperationPhase::APPLYING
                : ControlOperationPhase::ABORTED;
            m_activeOperationFingerprint = fingerprint;
            m_activeOperationBaseRevision = baseRevision;
            m_activeOperationTargetRevision = targetRevision;
        }

        const ControlOperationPhase preparationPhase = replayPersistedDocument
            ? ControlOperationPhase::APPLYING : ControlOperationPhase::ABORTED;
        const esp_err_t preparationResult = replayPersistedDocument
            ? ESP_OK : ESP_ERR_INVALID_STATE;
        const esp_err_t preparationJournal = persistOperationJournal(
            operationId, fingerprint, baseRevision, targetRevision,
            preparationPhase, preparationResult);
        if (preparationJournal != ESP_OK) {
            {
                std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
                m_activeOperationId = 0;
                m_activeOperationPhase = ControlOperationPhase::IDLE;
            }
            if (m_operationGate && reservation.valid()) {
                (void)m_operationGate->release(
                    reservation, ControlOperationPhase::FAILED,
                    preparationJournal);
            }
            return preparationJournal;
        }

        if (replayPersistedDocument) {
            // The persisted document is the source of truth after a reset.
            // The synchronous EventBus publication re-applies it to every
            // runtime consumer before the operation is marked committed.
            m_configChangePublisher.publishFullConfig(currentConfig);
        }
        const ControlOperationPhase terminalPhase = replayPersistedDocument
            ? ControlOperationPhase::SUCCEEDED : ControlOperationPhase::ABORTED;
        const esp_err_t terminalResult = replayPersistedDocument
            ? ESP_OK : ESP_ERR_INVALID_STATE;
        const esp_err_t journalResult = persistOperationJournal(
            operationId, fingerprint, baseRevision, targetRevision,
            terminalPhase, terminalResult);
        {
            std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
            m_activeOperationId = 0;
            m_activeOperationPhase = ControlOperationPhase::IDLE;
            if (journalResult == ESP_OK) {
                m_lastOperationId = operationId;
                m_lastOperationFingerprint = fingerprint;
                m_lastOperationResult = terminalResult;
                m_lastOperationPhase = terminalPhase;
                m_lastOperationBaseRevision = baseRevision;
                m_lastOperationTargetRevision = targetRevision;
                m_hasLastOperation = true;
                m_operationRecoveryPending = false;
                m_recoveryOperationId = 0;
                m_recoveryOperationFingerprint = 0;
                m_recoveryBaseRevision = 0;
                m_recoveryTargetRevision = 0;
            }
        }
        if (journalResult == ESP_OK) {
            if (m_operationGate) {
                m_operationGate->setRecoveryPending(false);
                if (reservation.valid()) {
                    (void)m_operationGate->release(
                        reservation, terminalPhase, terminalResult);
                }
            }
            // The acknowledgement request itself completed successfully even
            // when the interrupted transaction is deterministically recorded
            // as ABORTED.  Expose that operation result through the status
            // record, but do not turn a successful recovery action into an
            // HTTP error that leaves clients believing the latch is still
            // pending.
            return ESP_OK;
        }
        if (m_operationGate && reservation.valid()) {
            (void)m_operationGate->release(reservation,
                                           ControlOperationPhase::FAILED,
                                           journalResult);
        }
        return journalResult;
    }
}

esp_err_t ConfigurationService::init() {
    std::lock_guard<std::mutex> operationLock(m_operationMutex);
    const esp_err_t journalRet = loadOperationJournal();
    if (journalRet != ESP_OK && journalRet != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Configuration operation recovery requires explicit reconciliation");
    }
    ESP_LOGI(TAG, "Initializing Configuration Service. Loading from key '%s'", m_configKey.c_str());
    std::string rawData;
    ConfigData loadedConfig;
    esp_err_t ret = m_storageService.loadData(m_configKey, rawData);
    bool useDefaults = false;
    bool persistentConfigRecoveryRequired = false;
    std::string reasonForDefaults;

    if (ret == ESP_OK) {
        if (rawData.empty()) {
            reasonForDefaults = "Config file empty";
            useDefaults = true;
            persistentConfigRecoveryRequired = true;
        } else {
            ret = m_configParser.deserialize(rawData, loadedConfig);
            if (ret == ESP_OK) {
                std::string validationError;
                if (!m_configValidator.validate(loadedConfig, validationError)) {
                    reasonForDefaults = "Validation failed: " + validationError;
                    useDefaults = true;
                    persistentConfigRecoveryRequired = true;
                } else {
                    ESP_LOGI(TAG, "Configuration loaded, parsed, and validated successfully (Version: %d).", loadedConfig.config_version);
                }
            } else {
                reasonForDefaults = "JSON parse failed";
                useDefaults = true;
                persistentConfigRecoveryRequired = true;
            }
        }
    } else if (ret == ESP_ERR_NOT_FOUND) {
        reasonForDefaults = m_storageService.isFreshStorage()
            ? "Config file not found on fresh storage"
            : "Config file missing from existing storage";
        useDefaults = true;
        // ENOENT on a demonstrably existing medium is an unknown recovery
        // condition.  Only a backend that positively identified a fresh
        // partition may take the normal first-boot defaults path.
        persistentConfigRecoveryRequired = !m_storageService.isFreshStorage();
    } else {
        reasonForDefaults = std::string("Storage load failed: ") + esp_err_to_name(ret);
        useDefaults = true;
        persistentConfigRecoveryRequired = true;
    }

    if (useDefaults) {
        ESP_LOGW(TAG, "%s. Using default values.", reasonForDefaults.c_str());
        loadedConfig = ConfigData(); // Re-assign defaults in memory only.
        ESP_LOGW(TAG, "Stored configuration preserved; defaults are in memory only.");
    }

    if (persistentConfigRecoveryRequired) {
        // A present but unreadable/invalid document is not equivalent to a
        // first boot. Keep the application available for an explicit v3
        // repair POST, but mark the operation as unknown so no default
        // strategy can arm the motors and no stale journal identity can be
        // used to acknowledge the wrong in-memory defaults.  This deliberately
        // supersedes even a syntactically valid journal: the main document is
        // the configuration authority, and it was not recovered.
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        m_operationRecoveryPending = true;
        m_recoveryOperationId = 0;
        m_recoveryOperationFingerprint = 0;
        m_recoveryBaseRevision = loadedConfig.config_revision;
        m_recoveryTargetRevision = loadedConfig.config_revision;
        m_lastOperationResult = ESP_ERR_INVALID_STATE;
        m_lastOperationPhase = ControlOperationPhase::ABORTED;
        m_hasLastOperation = false;
        if (m_operationGate) m_operationGate->setRecoveryPending(true);
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_configData = loadedConfig;
    }

    // Publish initial config state regardless of load source
    ESP_LOGI(TAG, "Publishing initial configuration state.");
    m_configChangePublisher.publishFullConfig(loadedConfig);

    return ESP_OK; // Return OK even if defaults were used, as the service is operational
}

bool ConfigurationService::tryReserveConfiguration(
    ControlOperationReservation& reservation,
    const char* operation,
    uint64_t requestedOperationId)
{
    if (!m_operationGate) {
        return true;
    }
    if (m_operationGate->configurationRecoveryPending()) {
        bool isRecoveryOwner = false;
        {
            std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
            const bool unknownRecovery = m_operationRecoveryPending &&
                m_recoveryOperationId == 0;
            isRecoveryOwner = m_operationRecoveryPending &&
                requestedOperationId != 0 &&
                (unknownRecovery || requestedOperationId == m_recoveryOperationId);
        }
        if (!isRecoveryOwner) {
            ESP_LOGW(TAG, "Rejecting %s until configuration operation recovery is acknowledged",
                     operation ? operation : "configuration operation");
            return false;
        }
    }
    if (m_operationGate->tryAcquire(ControlOperationKind::CONFIGURATION,
                                    reservation,
                                    requestedOperationId)) {
        m_operationGate->updatePhase(reservation,
                                     ControlOperationPhase::PREPARING);
        return true;
    }
    ESP_LOGW(TAG, "Rejecting %s because another control operation is reserved",
             operation ? operation : "configuration operation");
    return false;
}

uint64_t ConfigurationService::beginConfigurationOperation(
    const ControlOperationReservation& reservation,
    uint64_t fingerprint,
    uint32_t baseRevision,
    uint32_t targetRevision,
    uint64_t requestedOperationId)
{
    uint64_t operationId = requestedOperationId;
    std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
    if (operationId == 0) {
        operationId = reservation.valid() ? reservation.operationId
                                          : ++m_localOperationSequence;
        if (operationId == 0) {
            operationId = ++m_localOperationSequence;
        }
    }
    m_activeOperationId = operationId;
    m_activeOperationPhase = ControlOperationPhase::PREPARING;
    m_activeOperationFingerprint = fingerprint;
    m_activeOperationBaseRevision = baseRevision;
    m_activeOperationTargetRevision = targetRevision;
    return operationId;
}

esp_err_t ConfigurationService::completeConfigurationOperation(
    ControlOperationReservation& reservation,
    uint64_t operationId,
    uint64_t fingerprint,
    uint32_t baseRevision,
    uint32_t targetRevision,
    ControlOperationPhase phase,
    esp_err_t result)
{
    ControlOperationPhase previousPhase = ControlOperationPhase::IDLE;
    {
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        if (m_activeOperationId == operationId) {
            previousPhase = m_activeOperationPhase;
        }
    }
    setActiveOperationPhase(operationId, phase);
    const esp_err_t journalResult = persistOperationJournal(
        operationId, fingerprint, baseRevision, targetRevision, phase, result);
    if (m_operationGate && reservation.valid()) {
        (void)m_operationGate->updatePhase(reservation, phase, result);
    }
    {
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        m_activeOperationId = 0;
        m_activeOperationPhase = ControlOperationPhase::IDLE;
        m_lastOperationId = operationId;
        m_lastOperationFingerprint = fingerprint;
        m_lastOperationResult = journalResult == ESP_OK ? result : journalResult;
        m_lastOperationPhase = journalResult == ESP_OK
            ? phase : ControlOperationPhase::ABORTED;
        m_lastOperationBaseRevision = baseRevision;
        m_lastOperationTargetRevision = targetRevision;
        m_hasLastOperation = true;
        const bool writeWasStarted = previousPhase == ControlOperationPhase::WRITING ||
            previousPhase == ControlOperationPhase::APPLYING ||
            phase == ControlOperationPhase::APPLYING;
        if (journalResult != ESP_OK ||
            (result != ESP_OK && writeWasStarted &&
             !m_lastPersistenceFailureResolved)) {
            m_operationRecoveryPending = true;
            m_recoveryOperationId = operationId;
            m_recoveryOperationFingerprint = fingerprint;
            m_recoveryBaseRevision = baseRevision;
            m_recoveryTargetRevision = targetRevision;
        } else if (phase == ControlOperationPhase::SUCCEEDED) {
            m_operationRecoveryPending = false;
            m_recoveryOperationId = 0;
            m_recoveryOperationFingerprint = 0;
            m_recoveryBaseRevision = 0;
            m_recoveryTargetRevision = 0;
        }
    }
    bool recoveryPending = false;
    {
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        recoveryPending = m_operationRecoveryPending;
    }
    if (m_operationGate) {
        // A failed/uncertain operation must not clear an older unknown
        // recovery latch merely because its terminal journal write happened
        // to succeed.
        m_operationGate->setRecoveryPending(recoveryPending);
    }
    if (m_operationGate && reservation.valid()) {
        (void)m_operationGate->release(
            reservation,
            journalResult == ESP_OK ? phase : ControlOperationPhase::FAILED,
            journalResult == ESP_OK ? result : journalResult);
    }
    return journalResult == ESP_OK ? result : journalResult;
}

void ConfigurationService::setActiveOperationPhase(
    uint64_t operationId,
    ControlOperationPhase phase)
{
    std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
    if (m_activeOperationId == operationId) {
        m_activeOperationPhase = phase;
    }
}

esp_err_t ConfigurationService::save() {
    ControlOperationReservation reservation;
    if (!tryReserveConfiguration(reservation, "save")) {
        return ESP_ERR_INVALID_STATE;
    }
    std::lock_guard<std::mutex> operationLock(m_operationMutex);
    ConfigData snapshot;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        snapshot = m_configData;
    }
    const uint32_t baseRevision = snapshot.config_revision;
    const uint64_t fingerprint = stableHash(
        "save:" + std::to_string(baseRevision));
    const uint64_t operationId = beginConfigurationOperation(
        reservation, fingerprint, baseRevision, baseRevision);
    if (persistOperationJournal(operationId, fingerprint, baseRevision,
                                baseRevision,
                                ControlOperationPhase::PREPARING, 0) != ESP_OK) {
        return completeConfigurationOperation(
            reservation, operationId, fingerprint, baseRevision, baseRevision,
            ControlOperationPhase::FAILED, ESP_FAIL);
    }

    esp_err_t result = ESP_OK;
    setActiveOperationPhase(operationId, ControlOperationPhase::WRITING);
    if (m_operationGate && reservation.valid()) {
        (void)m_operationGate->updatePhase(
            reservation, ControlOperationPhase::WRITING);
    }
    const esp_err_t journalWriteRet = persistOperationJournal(
        operationId, fingerprint, baseRevision, baseRevision,
        ControlOperationPhase::WRITING, 0);
    result = journalWriteRet == ESP_OK ? saveInternal(snapshot) : journalWriteRet;
    if (result != ESP_OK) {
        return completeConfigurationOperation(
            reservation, operationId, fingerprint, baseRevision, baseRevision,
            ControlOperationPhase::FAILED, result);
    }

    const esp_err_t journalApplyRet = persistOperationJournal(
        operationId, fingerprint, baseRevision, baseRevision,
        ControlOperationPhase::APPLYING, 0);
    if (m_operationGate && reservation.valid()) {
        setActiveOperationPhase(operationId, ControlOperationPhase::APPLYING);
        (void)m_operationGate->updatePhase(
            reservation, ControlOperationPhase::APPLYING);
    }
    return completeConfigurationOperation(
        reservation, operationId, fingerprint, baseRevision, baseRevision,
        journalApplyRet == ESP_OK ? ControlOperationPhase::SUCCEEDED
                                  : ControlOperationPhase::FAILED,
        journalApplyRet == ESP_OK ? ESP_OK : journalApplyRet);
}

esp_err_t ConfigurationService::updateConfigFromJson(const std::string& json, std::string* error) {
    const uint64_t fingerprint = requestFingerprint(json);
    uint64_t requestedOperationId = 0;
    // Legacy documents are still accepted by JsonConfigParser for boot-time
    // migration, but an HTTP read/modify/write must carry the v3 schema and a
    // document revision. Otherwise a stale v1/v2 client could erase the
    // inactive strategy set while its revision still happens to be zero.
    if (cJSON* requestRoot = cJSON_Parse(json.c_str())) {
        cJSON* operationIdItem = cJSON_GetObjectItem(requestRoot, "operation_id");
        if (!operationIdItem) {
            operationIdItem = cJSON_GetObjectItem(requestRoot, "operationId");
        }
        if (!parseOperationId(operationIdItem, requestedOperationId)) {
            if (error) *error = "operation_id must be a positive decimal integer or string";
            cJSON_Delete(requestRoot);
            return ESP_ERR_INVALID_ARG;
        }
        cJSON* version = cJSON_GetObjectItem(requestRoot, "config_version");
        cJSON* documentRevision = cJSON_GetObjectItem(requestRoot, "config_revision");
        const bool legacy = !version ||
            (cJSON_IsNumber(version) && std::isfinite(version->valuedouble) &&
             (version->valuedouble == 1.0 || version->valuedouble == 2.0));
        if (legacy) {
            if (error) *error = "Legacy configuration POST is not accepted; read the v3 document before saving";
            cJSON_Delete(requestRoot);
            return ESP_ERR_NOT_SUPPORTED;
        }
        if (!documentRevision) {
            if (error) *error = "Configuration POST must include config_revision from the loaded document";
            cJSON_Delete(requestRoot);
            return ESP_ERR_INVALID_ARG;
        }
        cJSON_Delete(requestRoot);
    }

    if (requestedOperationId != 0) {
        if (m_operationGate) {
            const ControlOperationStatus current = m_operationGate->currentStatus();
            if (current.active && current.operationId == requestedOperationId) {
                if (error) *error = "This configuration operation is still in progress";
                return ESP_ERR_INVALID_STATE;
            }
        }
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        if (m_activeOperationId == requestedOperationId) {
            if (error) *error = "This configuration operation is still in progress";
            return ESP_ERR_INVALID_STATE;
        }
        if (!m_operationRecoveryPending && m_hasLastOperation &&
            m_lastOperationId == requestedOperationId) {
            if (m_lastOperationFingerprint == fingerprint) {
                if (error && m_lastOperationResult != ESP_OK) {
                    *error = "The previous configuration operation failed";
                }
                return m_lastOperationResult;
            }
            if (error) *error = "operation_id was already used for a different payload";
            return ESP_ERR_INVALID_STATE;
        }
    }

    ConfigData tempConfig; // Create a temporary config to parse into
    esp_err_t ret = m_configParser.deserialize(json, tempConfig);
    if (ret != ESP_OK) {
        if (error) *error = ret == ESP_ERR_NOT_SUPPORTED ? "Unsupported config_version; supported versions are 1, 2 and 3" : "Invalid configuration JSON or field type";
        ESP_LOGE(TAG, "Failed to deserialize JSON for update: %s", esp_err_to_name(ret));
        return ret;
    }

    std::string validationError;
    if (!m_configValidator.validate(tempConfig, validationError)) {
        if (error) *error = validationError;
        ESP_LOGE(TAG, "Validation failed for config update: %s", validationError.c_str());
        return ESP_FAIL; // Return specific validation failure (maybe a different error code?)
    }

    // If a reset interrupted the apply half of the previous transaction, the
    // only safe replay is the exact same operation and payload. It confirms
    // the already persisted document and republishes it; a different write
    // must wait for that reconciliation.
    bool recoveryReplay = false;
    bool unknownRecoveryRepair = false;
    {
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        if (m_operationRecoveryPending) {
            const bool unknownRecovery = m_recoveryOperationId == 0;
            if (requestedOperationId == 0 ||
                (!unknownRecovery &&
                 (requestedOperationId != m_recoveryOperationId ||
                  fingerprint != m_recoveryOperationFingerprint))) {
                if (error) *error = "An interrupted configuration operation must be reconciled first";
                return ESP_ERR_INVALID_STATE;
            }
            // With no trustworthy operation identity (for example a
            // malformed journal or a corrupt main document), a complete v3
            // document with an explicit new operation id is the only repair
            // path. It is validated and revision-checked below, then commits
            // through the same transaction as a normal full update.
            recoveryReplay = !unknownRecovery;
            unknownRecoveryRepair = unknownRecovery;
        }
    }
    if (recoveryReplay) {
        return acknowledgeOperation(requestedOperationId);
    }

    // Store old config to detect changes
    ConfigData oldConfig;
    ConfigData newConfig;
    ControlOperationReservation reservation;
    if (!tryReserveConfiguration(reservation, "full configuration update",
                                 requestedOperationId)) {
        if (error) *error = "Configuration operation is busy; retry after the active operation finishes";
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t operationId = requestedOperationId;
    {
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        if (operationId == 0) {
            operationId = reservation.valid() ? reservation.operationId
                                              : ++m_localOperationSequence;
            if (operationId == 0) {
                operationId = ++m_localOperationSequence;
            }
        }
        m_activeOperationId = operationId;
        m_activeOperationPhase = ControlOperationPhase::PREPARING;
    }
    const auto completeOperation = [&](esp_err_t result,
                                       ControlOperationPhase phase) -> esp_err_t {
        ControlOperationPhase previousPhase = ControlOperationPhase::IDLE;
        {
            std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
            if (m_activeOperationId == operationId) {
                previousPhase = m_activeOperationPhase;
            }
        }
        setActiveOperationPhase(operationId, phase);
        const esp_err_t journalResult = persistOperationJournal(
            operationId,
            fingerprint,
            m_activeOperationBaseRevision,
            m_activeOperationTargetRevision,
            phase,
            result);
        if (m_operationGate && reservation.valid()) {
            (void)m_operationGate->updatePhase(reservation, phase, result);
        }
        {
            std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
            m_activeOperationId = 0;
            m_activeOperationPhase = ControlOperationPhase::IDLE;
            m_lastOperationId = operationId;
            m_lastOperationFingerprint = fingerprint;
            m_lastOperationResult = journalResult == ESP_OK ? result : journalResult;
            m_lastOperationPhase = journalResult == ESP_OK
                ? phase : ControlOperationPhase::ABORTED;
            m_lastOperationBaseRevision = m_activeOperationBaseRevision;
            m_lastOperationTargetRevision = m_activeOperationTargetRevision;
            m_hasLastOperation = true;
            const bool writeWasStarted = previousPhase == ControlOperationPhase::WRITING ||
                previousPhase == ControlOperationPhase::APPLYING ||
                phase == ControlOperationPhase::APPLYING;
            if (journalResult != ESP_OK ||
                (result != ESP_OK && writeWasStarted &&
                 !m_lastPersistenceFailureResolved)) {
                // The config file may already have been published while the
                // journal write failed. Keep the safety latch until the exact
                // operation can be replayed or acknowledged.
                m_operationRecoveryPending = true;
                m_recoveryOperationId = operationId;
                m_recoveryOperationFingerprint = fingerprint;
                m_recoveryBaseRevision = m_activeOperationBaseRevision;
                m_recoveryTargetRevision = m_activeOperationTargetRevision;
            } else if (phase == ControlOperationPhase::SUCCEEDED) {
                m_operationRecoveryPending = false;
                m_recoveryOperationId = 0;
                m_recoveryOperationFingerprint = 0;
                m_recoveryBaseRevision = 0;
                m_recoveryTargetRevision = 0;
            }
        }
        bool recoveryPending = false;
        {
            std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
            recoveryPending = m_operationRecoveryPending;
        }
        if (m_operationGate) {
            m_operationGate->setRecoveryPending(recoveryPending);
        }
        if (m_operationGate && reservation.valid()) {
            (void)m_operationGate->release(
                reservation,
                journalResult == ESP_OK ? phase : ControlOperationPhase::FAILED,
                journalResult == ESP_OK ? result : journalResult);
        }
        return journalResult == ESP_OK ? result : journalResult;
    };
    std::lock_guard<std::mutex> operationLock(m_operationMutex);

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_activeOperationBaseRevision = m_configData.config_revision;
        m_activeOperationTargetRevision =
            m_activeOperationBaseRevision < std::numeric_limits<uint32_t>::max()
                ? m_activeOperationBaseRevision + 1
                : m_activeOperationBaseRevision;
    }
    if (persistOperationJournal(operationId, fingerprint,
                                m_activeOperationBaseRevision,
                                m_activeOperationTargetRevision,
                                ControlOperationPhase::PREPARING,
                                0) != ESP_OK) {
        if (error) *error = "Unable to persist configuration operation journal";
        completeOperation(ESP_FAIL, ControlOperationPhase::FAILED);
        return ESP_FAIL;
    }
    
    // A v3 client must send the revision it read.  Prepare the candidate
    // under the short runtime mutex, then release it before journal/storage
    // I/O.  The operation mutex and gate exclude another configuration
    // mutation while this candidate is being persisted.
    bool candidateReady = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        oldConfig = m_configData;
        const bool revisionMatches = tempConfig.config_revision ==
            oldConfig.config_revision;
        const bool repairRevisionAdvances = unknownRecoveryRepair &&
            tempConfig.config_revision > oldConfig.config_revision;
        // An unknown recovery has no trustworthy base document.  Accept only
        // an explicit, complete repair whose revision advances the last RAM
        // snapshot; accepting an equal revision would let a rejected repair
        // clear the existing safety latch.
        const bool revisionAccepted = unknownRecoveryRepair
            ? repairRevisionAdvances : revisionMatches;
        if (!revisionAccepted) {
            if (error) *error = "Configuration document revision conflict; reload before saving";
            ESP_LOGW(TAG, "Rejected config update with stale document revision (%" PRIu32 ", current %" PRIu32 ")",
                     tempConfig.config_revision, oldConfig.config_revision);
            ret = ESP_ERR_INVALID_STATE;
        } else if (m_controlActive &&
                   tempConfig.control.strategies.active != oldConfig.control.strategies.active) {
            if (error) *error = "Strategy changes require an inactive control mode";
            ESP_LOGW(TAG, "Rejected strategy change while control mode is active");
            ret = ESP_ERR_INVALID_STATE;
        } else if (!unknownRecoveryRepair &&
                   tempConfig.control.strategies.revision != oldConfig.control.strategies.revision) {
            if (error) *error = "Configuration revision conflict; reload before saving";
            ESP_LOGW(TAG, "Rejected config update with stale strategy revision (%" PRIu32 ", current %" PRIu32 ")",
                     tempConfig.control.strategies.revision,
                     oldConfig.control.strategies.revision);
            ret = ESP_ERR_INVALID_STATE;
        } else if (oldConfig.config_revision == std::numeric_limits<uint32_t>::max()) {
            if (error) *error = "Configuration document revision exhausted";
            ret = ESP_ERR_INVALID_STATE;
        } else {
            if (!unknownRecoveryRepair &&
                tempConfig.control.strategies.active != oldConfig.control.strategies.active) {
                tempConfig.control.strategies.revision = oldConfig.control.strategies.revision + 1;
            }
            if (!unknownRecoveryRepair &&
                tempConfig.control.strategies.nested_pid != oldConfig.control.strategies.nested_pid) {
                tempConfig.control.strategies.nested_pid.revision =
                    oldConfig.control.strategies.nested_pid.revision + 1;
                tempConfig.control.strategies.revision = oldConfig.control.strategies.revision + 1;
            }
            if (!unknownRecoveryRepair &&
                tempConfig.control.strategies.longitudinal_cascade != oldConfig.control.strategies.longitudinal_cascade) {
                tempConfig.control.strategies.longitudinal_cascade.revision =
                    oldConfig.control.strategies.longitudinal_cascade.revision + 1;
                tempConfig.control.strategies.revision = oldConfig.control.strategies.revision + 1;
            }
            if (!unknownRecoveryRepair) {
                tempConfig.config_revision = oldConfig.config_revision + 1;
            }
            newConfig = tempConfig;
            candidateReady = true;
        }
    }

    if (candidateReady && unknownRecoveryRepair) {
        // The repair document supplied by the client is already a complete
        // v3 snapshot. Preserve its monotonic revision and expose it in the
        // sidecar before the write begins; unlike a normal edit there is no
        // trustworthy base document to derive per-section revisions from.
        std::lock_guard<std::mutex> statusLock(m_operationStatusMutex);
        m_activeOperationTargetRevision = newConfig.config_revision;
    }

    if (!candidateReady) {
        completeOperation(ret, ControlOperationPhase::FAILED);
        return ret;
    }

    setActiveOperationPhase(operationId, ControlOperationPhase::WRITING);
    if (m_operationGate && reservation.valid()) {
        (void)m_operationGate->updatePhase(reservation,
                                            ControlOperationPhase::WRITING);
    }
    const esp_err_t journalWriteRet = persistOperationJournal(
        operationId, fingerprint, oldConfig.config_revision,
        newConfig.config_revision, ControlOperationPhase::WRITING, 0);
    ret = journalWriteRet == ESP_OK ? saveInternal(newConfig) : journalWriteRet;
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save configuration after update.");
    } else {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_configData.config_revision != oldConfig.config_revision) {
            ESP_LOGE(TAG, "Configuration changed while candidate was being persisted");
            ret = ESP_ERR_INVALID_STATE;
        } else {
            m_configData = newConfig;
            ESP_LOGI(TAG, "Configuration updated and saved successfully (Version: %d).",
                     m_configData.config_version);
        }
    }
    
    if (ret != ESP_OK) {
        completeOperation(ret, ControlOperationPhase::FAILED);
        return ret;
    }

    const esp_err_t journalApplyRet = persistOperationJournal(
        operationId, fingerprint, oldConfig.config_revision,
        newConfig.config_revision, ControlOperationPhase::APPLYING, 0);
    setActiveOperationPhase(operationId, ControlOperationPhase::APPLYING);
    if (m_operationGate && reservation.valid()) {
        (void)m_operationGate->updatePhase(reservation,
                                            ControlOperationPhase::APPLYING);
    }
    // Publish granular events for changed components
    m_configChangePublisher.publishChanges(oldConfig, newConfig);
    
    // Still publish the full update as a fallback for components not using specific events
    ESP_LOGI(TAG, "Publishing general configuration update event.");
    m_configChangePublisher.publishFullConfig(newConfig);

    const esp_err_t completionRet = completeOperation(
        journalApplyRet == ESP_OK ? ESP_OK : journalApplyRet,
        journalApplyRet == ESP_OK
            ? ControlOperationPhase::SUCCEEDED
            : ControlOperationPhase::FAILED);
    // A failure while recording the terminal journal state is part of the
    // transaction result.  Returning success here would tell an HTTP client
    // that the document was committed while the recovery latch is still set.
    if (completionRet != ESP_OK) {
        if (error) *error = "Configuration was written but operation commit requires reconciliation";
        return completionRet;
    }

    return ret;
}

esp_err_t ConfigurationService::applyPidConfig(const std::string& pidName,
                                                const PIDConfig& config,
                                                bool persist,
                                                uint64_t expectedConfigRevision) {
    ConfigData oldConfig;
    ConfigData newConfig;
    ControlOperationReservation reservation;
    if (!tryReserveConfiguration(reservation, "PID configuration update")) {
        return ESP_ERR_INVALID_STATE;
    }
    std::lock_guard<std::mutex> operationLock(m_operationMutex);

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        oldConfig = m_configData;
    }

    const uint32_t baseRevision = oldConfig.config_revision;
    const uint32_t targetRevision = baseRevision < std::numeric_limits<uint32_t>::max()
        ? baseRevision + 1 : baseRevision;
    const std::string fingerprintText =
        "pid:" + pidName + ":" +
        std::to_string(config.pid_kp) + ":" +
        std::to_string(config.pid_ki) + ":" +
        std::to_string(config.pid_kd) + ":" +
        std::to_string(config.pid_output_min) + ":" +
        std::to_string(config.pid_output_max) + ":" +
        std::to_string(config.pid_iterm_min) + ":" +
        std::to_string(config.pid_iterm_max);
    const uint64_t fingerprint = stableHash(fingerprintText);
    const uint64_t operationId = beginConfigurationOperation(
        reservation, fingerprint, baseRevision, targetRevision);

    if (persistOperationJournal(operationId, fingerprint, baseRevision,
                                targetRevision,
                                ControlOperationPhase::PREPARING, 0) != ESP_OK) {
        return completeConfigurationOperation(
            reservation, operationId, fingerprint, baseRevision, targetRevision,
            ControlOperationPhase::FAILED, ESP_FAIL);
    }

    esp_err_t ret = ESP_OK;
    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        oldConfig = m_configData;
        newConfig = m_configData;
        if (expectedConfigRevision != UINT64_MAX &&
            expectedConfigRevision != oldConfig.config_revision) {
            ESP_LOGW(TAG, "Rejected PID update for '%s' due to document revision conflict (expected %llu, current %" PRIu32 ")",
                     pidName.c_str(),
                     static_cast<unsigned long long>(expectedConfigRevision),
                     oldConfig.config_revision);
            ret = ESP_ERR_INVALID_STATE;
        } else {
            auto& nested = newConfig.control.strategies.nested_pid;
            if (pidName == "angle") {
                nested.angle = config;
            } else if (pidName == "speed_left") {
                nested.speed_left = config;
            } else if (pidName == "speed_right") {
                nested.speed_right = config;
            } else if (pidName == "yaw_angle") {
                nested.yaw_angle = config;
            } else if (pidName == "yaw_rate") {
                nested.yaw_rate = config;
            } else {
                ESP_LOGE(TAG, "Unknown PID name for applyPidConfig: %s", pidName.c_str());
                ret = ESP_ERR_INVALID_ARG;
            }

            changed = ret == ESP_OK &&
                newConfig.control.strategies.nested_pid !=
                oldConfig.control.strategies.nested_pid;
            if (ret == ESP_OK && changed) {
                if (oldConfig.config_revision == std::numeric_limits<uint32_t>::max()) {
                    ret = ESP_ERR_INVALID_STATE;
                } else {
                    newConfig.control.strategies.nested_pid.revision =
                        oldConfig.control.strategies.nested_pid.revision + 1;
                    newConfig.control.strategies.revision =
                        oldConfig.control.strategies.revision + 1;
                    newConfig.config_revision = oldConfig.config_revision + 1;
                }
            }
        }
    }

    if (ret == ESP_OK) {
        std::string validationError;
        if (!m_configValidator.validate(newConfig, validationError)) {
            ESP_LOGE(TAG, "Rejected PID update for '%s': %s",
                     pidName.c_str(), validationError.c_str());
            ret = ESP_FAIL;
        }
    }

    if (ret == ESP_OK && persist && changed) {
        setActiveOperationPhase(operationId, ControlOperationPhase::WRITING);
        if (m_operationGate && reservation.valid()) {
            (void)m_operationGate->updatePhase(
                reservation, ControlOperationPhase::WRITING);
        }
        const esp_err_t journalWriteRet = persistOperationJournal(
            operationId, fingerprint, oldConfig.config_revision,
            newConfig.config_revision,
            ControlOperationPhase::WRITING, 0);
        ret = journalWriteRet == ESP_OK ? saveInternal(newConfig) : journalWriteRet;
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to persist PID config '%s'",
                     pidName.c_str());
        }
    }

    if (ret == ESP_OK) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_configData.config_revision != oldConfig.config_revision) {
            ESP_LOGE(TAG, "Configuration changed while PID candidate was being prepared");
            ret = ESP_ERR_INVALID_STATE;
        } else {
            // Commit only after storage succeeds.  For a non-persistent
            // tuning candidate this is still the in-memory apply path.
            m_configData = newConfig;
        }
    }

    if (ret != ESP_OK) {
        return completeConfigurationOperation(
            reservation, operationId, fingerprint, baseRevision, targetRevision,
            ControlOperationPhase::FAILED, ret);
    }

    const esp_err_t journalApplyRet = persistOperationJournal(
        operationId, fingerprint, oldConfig.config_revision,
        newConfig.config_revision, ControlOperationPhase::APPLYING, 0);
    setActiveOperationPhase(operationId, ControlOperationPhase::APPLYING);
    if (m_operationGate && reservation.valid()) {
        (void)m_operationGate->updatePhase(
            reservation, ControlOperationPhase::APPLYING);
    }
    if (changed) {
        m_configChangePublisher.publishChanges(oldConfig, newConfig);
        m_configChangePublisher.publishFullConfig(newConfig);
    }

    return completeConfigurationOperation(
        reservation, operationId, fingerprint, oldConfig.config_revision,
        newConfig.config_revision,
        journalApplyRet == ESP_OK ? ControlOperationPhase::SUCCEEDED
                                  : ControlOperationPhase::FAILED,
        journalApplyRet == ESP_OK ? ESP_OK : journalApplyRet);
}

void ConfigurationService::updateImuGyroOffsets(float x, float y, float z) {
    ESP_LOGI(TAG, "Updating IMU gyro offsets: x=%.2f, y=%.2f, z=%.2f", x, y, z);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ESP_LOGW(TAG, "Ignoring non-finite IMU gyro offsets");
        return;
    }

    ConfigData oldConfig;
    ConfigData newConfig;
    ControlOperationReservation reservation;
    const bool calibrationWrite = m_operationGate &&
        m_operationGate->isHeldBy(ControlOperationKind::CALIBRATION);
    if (!calibrationWrite &&
        !tryReserveConfiguration(reservation, "IMU gyro offset update")) {
        return;
    }
    std::lock_guard<std::mutex> operationLock(m_operationMutex);

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        oldConfig = m_configData;
    }
    const uint32_t baseRevision = oldConfig.config_revision;
    const uint32_t targetRevision = baseRevision < std::numeric_limits<uint32_t>::max()
        ? baseRevision + 1 : baseRevision;
    const std::string fingerprintText =
        "imu-offsets:" + std::to_string(x) + ":" +
        std::to_string(y) + ":" + std::to_string(z);
    const uint64_t fingerprint = stableHash(fingerprintText);
    uint64_t requestedOperationId = 0;
    if (calibrationWrite) {
        const ControlOperationStatus status = m_operationGate->currentStatus();
        if (status.active && status.kind == ControlOperationKind::CALIBRATION) {
            requestedOperationId = status.operationId;
        }
    }
    const uint64_t operationId = beginConfigurationOperation(
        reservation, fingerprint, baseRevision, targetRevision,
        requestedOperationId);
    if (persistOperationJournal(operationId, fingerprint, baseRevision,
                                targetRevision,
                                ControlOperationPhase::PREPARING, 0) != ESP_OK) {
        (void)completeConfigurationOperation(
            reservation, operationId, fingerprint, baseRevision, targetRevision,
            ControlOperationPhase::FAILED, ESP_FAIL);
        return;
    }

    esp_err_t ret = ESP_OK;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        oldConfig = m_configData;
        newConfig = oldConfig;
        if (oldConfig.config_revision == std::numeric_limits<uint32_t>::max()) {
            ret = ESP_ERR_INVALID_STATE;
        } else {
            newConfig.imu.gyro_offset_x = x;
            newConfig.imu.gyro_offset_y = y;
            newConfig.imu.gyro_offset_z = z;
            ++newConfig.config_revision;
        }
    }
    if (ret == ESP_OK) {
        setActiveOperationPhase(operationId, ControlOperationPhase::WRITING);
        if (m_operationGate && reservation.valid()) {
            (void)m_operationGate->updatePhase(
                reservation, ControlOperationPhase::WRITING);
        }
        const esp_err_t journalWriteRet = persistOperationJournal(
            operationId, fingerprint, oldConfig.config_revision,
            newConfig.config_revision,
            ControlOperationPhase::WRITING, 0);
        ret = journalWriteRet == ESP_OK ? saveInternal(newConfig) : journalWriteRet;
    }
    if (ret == ESP_OK) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_configData.config_revision != oldConfig.config_revision) {
            ret = ESP_ERR_INVALID_STATE;
        } else {
            m_configData = newConfig;
        }
    }
    if (ret != ESP_OK) {
        (void)completeConfigurationOperation(
            reservation, operationId, fingerprint, baseRevision, targetRevision,
            ControlOperationPhase::FAILED, ret);
        return;
    }
    ESP_LOGI(TAG, "Gyro offsets saved to storage");

    const esp_err_t journalApplyRet = persistOperationJournal(
        operationId, fingerprint, oldConfig.config_revision,
        newConfig.config_revision, ControlOperationPhase::APPLYING, 0);
    setActiveOperationPhase(operationId, ControlOperationPhase::APPLYING);
    if (m_operationGate && reservation.valid()) {
        (void)m_operationGate->updatePhase(
            reservation, ControlOperationPhase::APPLYING);
    }
    m_configChangePublisher.publishImuConfig(newConfig.imu, false);
    m_configChangePublisher.publishFullConfig(newConfig);
    (void)completeConfigurationOperation(
        reservation, operationId, fingerprint, oldConfig.config_revision,
        newConfig.config_revision,
        journalApplyRet == ESP_OK ? ControlOperationPhase::SUCCEEDED
                                  : ControlOperationPhase::FAILED,
        journalApplyRet == ESP_OK ? ESP_OK : journalApplyRet);
}

// EventHandler implementation
void ConfigurationService::handleEvent(const BaseEvent& event) {
    if (event.is<IMU_GyroOffsetsUpdated>()) {
        const IMU_GyroOffsetsUpdated& offsetEvent = event.as<IMU_GyroOffsetsUpdated>();
        updateImuGyroOffsets(offsetEvent.x_dps, offsetEvent.y_dps, offsetEvent.z_dps);
    } else if (event.is<CONTROL_RunModeChanged>()) {
        handleRunModeChanged(event.as<CONTROL_RunModeChanged>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

void ConfigurationService::handleRunModeChanged(const CONTROL_RunModeChanged& event) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (event.armId < m_controlArmId) {
        return;
    }
    m_controlArmId = event.armId;
    m_controlActive = event.mode != ControlRunMode::DISABLED;
}

// Keep for backward compatibility
void ConfigurationService::subscribeToEvents(EventBus& bus) {
    ESP_LOGW(TAG, "ConfigurationService::subscribeToEvents is deprecated. Use EventBus::subscribe with EventHandler instead.");
}

esp_err_t ConfigurationService::getJsonString(std::string& jsonOutput) const {
    std::lock_guard<std::mutex> lock(m_mutex); // Lock for reading m_configData
    esp_err_t ret = m_configParser.serialize(m_configData, jsonOutput);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize current configuration.");
    }
    return ret;
}

esp_err_t ConfigurationService::saveInternal(const ConfigData& snapshot) {
    m_lastPersistenceFailureResolved = false;
    ESP_LOGD(TAG, "Saving configuration to storage key '%s'", m_configKey.c_str());
    std::string rawData;
    esp_err_t ret = m_configParser.serialize(snapshot, rawData);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize configuration for saving.");
        // No storage call was made, so this is a resolved preparation error.
        m_lastPersistenceFailureResolved = true;
        return ret;
    }
    std::string previousData;
    const esp_err_t previousReadRet =
        m_storageService.loadData(m_configKey, previousData);
    const bool previousDataAvailable = previousReadRet == ESP_OK;
    ret = m_storageService.saveData(m_configKey, rawData);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save configuration data to storage (%s).", esp_err_to_name(ret));
        // A backend may fail before publication or after a partial
        // publication.  Prefer its explicit backup restoration, then verify
        // the bytes currently readable from the main key.  Only an exact
        // match with the previously loaded record is safe to classify as a
        // resolved failure; otherwise the durable state is uncertain.
        (void)m_storageService.restoreBackup(m_configKey);
        if (previousDataAvailable) {
            std::string currentData;
            const esp_err_t currentReadRet =
                m_storageService.loadData(m_configKey, currentData);
            if (currentReadRet == ESP_OK && currentData == previousData) {
                m_lastPersistenceFailureResolved = true;
                ESP_LOGW(TAG, "Configuration write failed before durable change; previous record verified");
            }
        }
    } else {
        std::string persistedData;
        const esp_err_t readbackRet = m_storageService.loadData(m_configKey, persistedData);
        if (readbackRet != ESP_OK || persistedData != rawData) {
            ESP_LOGE(TAG, "Configuration readback failed after save (%s)",
                     readbackRet == ESP_OK ? "content-mismatch" : esp_err_to_name(readbackRet));
            // The SPIFFS backend retains the previous complete record until a
            // successful commit.  Restore and verify it before deciding
            // whether the service can continue without a recovery latch.
            const esp_err_t restoreRet = m_storageService.restoreBackup(m_configKey);
            if (restoreRet == ESP_OK) {
                std::string restoredData;
                const esp_err_t restoredReadRet =
                    m_storageService.loadData(m_configKey, restoredData);
                if (restoredReadRet == ESP_OK && previousDataAvailable &&
                    restoredData == previousData) {
                    ESP_LOGW(TAG, "Configuration rollback verified after readback failure");
                    m_lastPersistenceFailureResolved = true;
                    return readbackRet == ESP_OK ? ESP_FAIL : readbackRet;
                }
            }
            ESP_LOGE(TAG, "Configuration rollback could not be verified; recovery remains pending");
            return readbackRet == ESP_OK ? ESP_FAIL : readbackRet;
        }
        const esp_err_t commitRet = m_storageService.commitData(m_configKey);
        if (commitRet != ESP_OK) {
            ESP_LOGE(TAG, "Configuration backup cleanup failed: %s",
                     esp_err_to_name(commitRet));
            return commitRet;
        }
        ESP_LOGD(TAG, "Configuration saved and verified successfully.");
    }
    return ret;
}
