#include "TelemetryHandler.hpp"
#include "EventBus.hpp" // Include event bus for subscription
#include "CONFIG_FullConfigUpdate.hpp" // Include event definition
#include "TELEMETRY_Snapshot.hpp"
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Need full struct def for WebServerConfig
#include "HttpResponseUtils.hpp"
#include "cJSON.h"
#include <memory>
#include "esp_check.h"
#include "esp_log.h"
#include <cstring>
#include <algorithm> // For std::max
#include <cstdio>
#include <cmath>
#include <utility>

namespace {

// A response-sized JSON string can exceed the available heap even though the
// telemetry ring and the requested batch are both bounded.  Stream the
// response through a small fixed buffer instead.  No operation in this writer
// allocates memory, and a transport failure is reported to the caller.
class TelemetryChunkWriter {
public:
    explicit TelemetryChunkWriter(httpd_req_t* request) : m_request(request) {}

    bool append(const char* text) {
        return text != nullptr && append(text, std::strlen(text));
    }

    bool append(const char* data, size_t length) {
        if (!ok() || data == nullptr) {
            return false;
        }

        while (length > 0) {
            if (m_length == sizeof(m_buffer)) {
                if (!flush()) {
                    return false;
                }
            }
            const size_t available = sizeof(m_buffer) - m_length;
            const size_t copied = std::min(available, length);
            std::memcpy(m_buffer + m_length, data, copied);
            m_length += copied;
            data += copied;
            length -= copied;
        }
        return true;
    }

    bool appendFloat(double value) {
        char valueBuffer[40];
        const double finiteValue = std::isfinite(value) ? value : 0.0;
        const int written = std::snprintf(valueBuffer, sizeof(valueBuffer), "%.6g", finiteValue);
        if (written < 0 || written >= static_cast<int>(sizeof(valueBuffer))) {
            m_error = ESP_ERR_INVALID_ARG;
            return false;
        }
        return append(valueBuffer, static_cast<size_t>(written));
    }

    bool appendInteger(long long value) {
        char valueBuffer[32];
        const int written = std::snprintf(valueBuffer, sizeof(valueBuffer), "%lld", value);
        if (written < 0 || written >= static_cast<int>(sizeof(valueBuffer))) {
            m_error = ESP_ERR_INVALID_ARG;
            return false;
        }
        return append(valueBuffer, static_cast<size_t>(written));
    }

    bool appendUnsigned(unsigned long long value) {
        char valueBuffer[32];
        const int written = std::snprintf(valueBuffer, sizeof(valueBuffer), "%llu", value);
        if (written < 0 || written >= static_cast<int>(sizeof(valueBuffer))) {
            m_error = ESP_ERR_INVALID_ARG;
            return false;
        }
        return append(valueBuffer, static_cast<size_t>(written));
    }

    bool appendUnsignedString(unsigned long long value) {
        char valueBuffer[40];
        const int written = std::snprintf(valueBuffer, sizeof(valueBuffer), "\"%llu\"", value);
        if (written < 0 || written >= static_cast<int>(sizeof(valueBuffer))) {
            m_error = ESP_ERR_INVALID_ARG;
            return false;
        }
        return append(valueBuffer, static_cast<size_t>(written));
    }

    bool ok() const { return m_error == ESP_OK; }
    esp_err_t error() const { return m_error; }

    esp_err_t finish() {
        if (!ok()) {
            return m_error;
        }
        if (!flush()) {
            return m_error;
        }
        const esp_err_t ret = httpd_resp_send_chunk(m_request, nullptr, 0);
        if (ret != ESP_OK) {
            m_error = ret;
        }
        return m_error;
    }

private:
    bool flush() {
        if (!ok() || m_length == 0) {
            return ok();
        }
        const esp_err_t ret = httpd_resp_send_chunk(m_request, m_buffer, m_length);
        m_length = 0;
        if (ret != ESP_OK) {
            m_error = ret;
            return false;
        }
        return true;
    }

    static constexpr size_t BUFFER_SIZE = 1024;
    httpd_req_t* m_request;
    char m_buffer[BUFFER_SIZE]{};
    size_t m_length = 0;
    esp_err_t m_error = ESP_OK;
};

} // namespace

// Constructor takes initial WebServerConfig
TelemetryHandler::TelemetryHandler(const WebServerConfig& initialWebConfig) :
    // m_configService removed
    m_telemetry_buffer_max_size(100) // Default before applying initial config
{
    applyConfig(initialWebConfig); // Apply initial config
    ESP_LOGI(TAG, "TelemetryHandler constructed.");
}

// Apply config values
void TelemetryHandler::applyConfig(const WebServerConfig& config) {
    const size_t maxSize = std::min(MAX_BUFFER_SIZE,
        std::max((size_t)1, (size_t)config.telemetry_buffer_size));
    std::lock_guard<std::mutex> lock(m_telemetryMutex);
    m_telemetry_buffer_max_size = maxSize;
    ESP_LOGI(TAG, "Applied TelemetryHandler params: BufferSize=%zu", m_telemetry_buffer_max_size);
    // Trim buffer if it exceeds new max size.
    while (m_telemetryBuffer.size() > m_telemetry_buffer_max_size) {
        m_telemetryBuffer.pop_front();
    }
}

// EventHandler implementation
void TelemetryHandler::handleEvent(const BaseEvent& event) {
    if (event.is<CONFIG_FullConfigUpdate>()) {
        handleConfigUpdate(event.as<CONFIG_FullConfigUpdate>());
    } else if (event.is<TELEMETRY_Snapshot>()) {
        addTelemetrySnapshot(event.as<TELEMETRY_Snapshot>().snapshot);
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

// Handle config update event
void TelemetryHandler::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
    ESP_LOGD(TAG, "Handling config update event.");
    const size_t maxSize = std::min(MAX_BUFFER_SIZE,
        std::max((size_t)1, (size_t)event.configData.web.telemetry_buffer_size));
    std::lock_guard<std::mutex> lock(m_telemetryMutex);
    if (m_has_config_revision &&
        event.configData.config_revision < m_config_revision) {
        ESP_LOGW(TAG, "Ignoring stale telemetry config revision %lu (current %lu)",
                 static_cast<unsigned long>(event.configData.config_revision),
                 static_cast<unsigned long>(m_config_revision));
        return;
    }
    m_config_revision = event.configData.config_revision;
    m_has_config_revision = true;
    m_telemetry_buffer_max_size = maxSize;
    while (m_telemetryBuffer.size() > m_telemetry_buffer_max_size) {
        m_telemetryBuffer.pop_front();
    }
}

void TelemetryHandler::addTelemetrySnapshot(const TelemetryDataPoint& data) {
    std::lock_guard<std::mutex> lock(m_telemetryMutex);
    // Use the configured buffer size
    if (m_telemetryBuffer.size() >= m_telemetry_buffer_max_size) {
        m_telemetryBuffer.pop_front();
        if (m_dropped_samples != UINT32_MAX) ++m_dropped_samples;
    }
    m_telemetryBuffer.push_back(data);
}

esp_err_t TelemetryHandler::handleRequest(httpd_req_t *req) {
    ESP_LOGV(TAG, "Received request for /data");
    // Serialize one response at a time because the bounded snapshot is a
    // member buffer.  This mutex is independent of the producer/ring mutex,
    // so telemetry acquisition never waits on network I/O.
    std::lock_guard<std::mutex> responseLock(m_responseMutex);

    size_t dataCount = 0;
    uint32_t droppedSamples = 0;
    size_t remainingSamples = 0;
    // Note: Interval is no longer fetched from ConfigService here. If needed,
    // the interval should also be stored locally and updated via handleConfigUpdate.
    // For now, assuming the JS doesn't strictly *need* the interval from this specific response.

    {
        std::lock_guard<std::mutex> lock(m_telemetryMutex);
        dataCount = std::min(MAX_RESPONSE_BATCH_SIZE, m_telemetryBuffer.size());
        for (size_t i = 0; i < dataCount; ++i) {
            m_responseBuffer[i] = m_telemetryBuffer.front();
            m_telemetryBuffer.pop_front();
        }
        droppedSamples = m_dropped_samples;
        remainingSamples = m_telemetryBuffer.size();
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    TelemetryChunkWriter writer(req);
    bool formatFailed = false;
    formatFailed = !writer.append("{\"format_version\":4,\"dropped_samples\":") ||
                   !writer.appendUnsigned(droppedSamples) ||
                   !writer.append(",\"remaining_samples\":") ||
                   !writer.appendUnsigned(static_cast<unsigned long long>(remainingSamples)) ||
                   !writer.append(",\"data\":[");

    for (size_t i = 0; i < dataCount && !formatFailed; ++i) {
        const auto& point = m_responseBuffer[i];
        formatFailed = !writer.append(i == 0 ? "[" : ",[",
                                      i == 0 ? 1U : 2U);
        bool first = true;
        const auto separator = [&]() {
            if (!first && !writer.append(",")) {
                formatFailed = true;
                return false;
            }
            first = false;
            return true;
        };
        const auto appendFloat = [&](double value) {
            if (separator() && !writer.appendFloat(value)) formatFailed = true;
        };
        const auto appendNullableFloat = [&](double value, bool valid) {
            if (!separator()) return;
            if ((!valid && !writer.append("null")) ||
                (valid && !writer.appendFloat(value))) formatFailed = true;
        };
        const auto appendInteger = [&](long long value) {
            if (separator() && !writer.appendInteger(value)) formatFailed = true;
        };
        const auto appendUnsigned = [&](unsigned long long value) {
            if (separator() && !writer.appendUnsigned(value)) formatFailed = true;
        };
        const auto appendStringNumber = [&](unsigned long long value) {
            if (separator() && !writer.appendUnsignedString(value)) formatFailed = true;
        };
        const auto appendBool = [&](bool value) {
            if (separator() && !writer.append(value ? "true" : "false")) formatFailed = true;
        };

            // v3-compatible prefix (0..17), kept byte-for-byte in order.
            appendFloat(point.pitch_deg);
            appendFloat(point.speedLeft_dps);
            appendFloat(point.speedRight_dps);
            appendFloat(point.batteryVoltage);
            appendInteger(point.systemState);
            appendFloat(point.speedSetpointLeft_dps);
            appendFloat(point.speedSetpointRight_dps);
            appendFloat(point.desiredAngle_deg);
            appendFloat(point.yawAngle_deg);
            appendFloat(point.targetYawAngle_deg);
            appendFloat(point.yawRate_dps);
            appendFloat(point.targetYawRate_dps);
            appendBool(point.imuValid);
            appendNullableFloat(point.imuAgeMs, std::isfinite(point.imuAgeMs) && point.imuAgeMs >= 0.0f);
            appendUnsigned(point.imuGeneration);
            appendBool(point.encoderLeftValid);
            appendBool(point.encoderRightValid);
            appendBool(point.imuSampleRepeated);

            // Explicit strategy/control snapshot (indices 18..69).
            appendInteger(static_cast<long long>(point.timestamp_us));
            appendInteger(static_cast<long long>(point.strategyId));
            appendInteger(static_cast<long long>(point.loopMode));
            appendUnsigned(point.strategyRevision);
            appendUnsigned(point.configRevision);
            appendStringNumber(point.commandSessionId);
            appendUnsigned(point.controlGeneration);
            appendUnsigned(point.odometryGeneration);
            appendStringNumber(point.odometrySequence);
            appendInteger(static_cast<long long>(point.controlPhase));
            appendBool(point.controlValid);
            appendBool(point.targetPitchValid);
            appendBool(point.targetPitchClamped);
            appendBool(point.targetPitchRateLimited);
            appendBool(point.positionTargetValid);
            appendBool(point.positionHoldActive);
            appendBool(point.synchronizationTargetValid);
            appendBool(point.velocityLoopEnabled);
            appendBool(point.positionLoopEnabled);
            appendBool(point.synchronizationEnabled);
            appendBool(point.motionCommandValid);
            appendBool(point.motionCommandFresh);
            appendBool(point.velocityFeedbackValid);
            appendBool(point.velocityTargetClamped);
            appendBool(point.velocityOutputSaturated);
            appendBool(point.velocityAntiWindup);
            appendBool(point.pitchPidSaturated);
            appendBool(point.balanceSaturated);
            appendBool(point.mixerSaturated);
            appendBool(point.syncLimited);
            appendBool(point.motionRequestLimited);
            appendInteger(static_cast<long long>(point.phaseReason));
            appendFloat(point.targetPitch_deg);
            appendFloat(point.targetVelocityMps);
            appendFloat(point.commandVelocityMps);
            appendFloat(point.measuredVelocityMps);
            appendFloat(point.holdVelocityRequestMps);
            appendFloat(point.holdVelocityTargetMps);
            appendFloat(point.positionM);
            appendFloat(point.holdPositionM);
            appendFloat(point.positionErrorM);
            appendFloat(point.distanceDifferenceM);
            appendFloat(point.distanceDifferenceTargetM);
            appendFloat(point.syncVelocityDifferenceMps);
            appendFloat(point.requestedBalanceEffort);
            appendFloat(point.balanceEffort);
            appendFloat(point.requestedSyncEffort);
            appendFloat(point.syncEffort);
            appendFloat(point.leftEffort);
            appendFloat(point.rightEffort);
            appendBool(point.yawTargetValid);
            appendBool(point.yawControlAvailable);
            appendBool(point.motorCommitAttempted);
            appendBool(point.motorCommitSucceeded);
            appendInteger(static_cast<long long>(point.motorCommitResult));
            appendInteger(static_cast<long long>(point.faultReason));
            appendBool(point.faultLatched);
            appendStringNumber(point.imuSampleSequence);
            appendUnsigned(point.controlStepCostUs);
            appendBool(point.controlStepLate);
        if (!formatFailed && !writer.append("]")) formatFailed = true;
    }
    if (!formatFailed && !writer.append("]}")) formatFailed = true;

    esp_err_t final_ret = ESP_FAIL;
    if (!formatFailed && writer.ok()) {
        final_ret = writer.finish();
        if (final_ret == ESP_OK) {
            ESP_LOGV(TAG, "Sent %zu telemetry points.", dataCount);
        }
    } else {
        final_ret = writer.error();
        if (final_ret == ESP_OK) final_ret = ESP_FAIL;
        ESP_LOGE(TAG, "Failed to format/send telemetry response (%s)",
                 esp_err_to_name(final_ret));
    }

    if (final_ret != ESP_OK && dataCount > 0) {
        // Serialization or transport failure must not silently erase the
        // batch. Reinsert it at the front, accounting for any samples that
        // arrived while the response was being prepared.
        std::lock_guard<std::mutex> lock(m_telemetryMutex);
        for (size_t i = dataCount; i > 0; --i) {
            if (m_telemetryBuffer.size() >= m_telemetry_buffer_max_size) {
                m_telemetryBuffer.pop_back();
                if (m_dropped_samples != UINT32_MAX) ++m_dropped_samples;
            }
            m_telemetryBuffer.push_front(m_responseBuffer[i - 1]);
        }
    }
    return final_ret;
}
