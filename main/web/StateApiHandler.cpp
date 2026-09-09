// main/web/StateApiHandler.cpp
#include "StateApiHandler.hpp"
#include "IMUService.hpp"
#include "esp_timer.h"
#include "StateManager.hpp"
#include "BatteryService.hpp"
#include "PidTuningService.hpp"
#include "GuidedCalibrationService.hpp"
#include "ConfigurationService.hpp"
#include "OTAService.hpp"
#include "BaseEvent.hpp"
#include "HttpResponseUtils.hpp"
#include "cJSON.h"
#include <memory>
#include <string>
#include "esp_log.h"
#include "esp_http_server.h"

StateApiHandler::StateApiHandler(StateManager& stateManager,
                                 BatteryService& batteryService,
                                 PidTuningService& pidTuningService,
                                 GuidedCalibrationService& guidedCalibrationService,
                                 ConfigurationService& configService,
                                 OTAService& otaService, IMUService& imuService)
    : m_imuService(imuService), m_stateManager(stateManager),
      m_batteryService(batteryService),
      m_pidTuningService(pidTuningService),
      m_guidedCalibrationService(guidedCalibrationService),
      m_configService(configService),
      m_otaService(otaService) {
    ESP_LOGI(TAG, "StateApiHandler constructed.");
}

// EventHandler implementation
void StateApiHandler::handleEvent(const BaseEvent& event) {
    ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
             getHandlerName().c_str(), event.eventName());
}

static const char* pidTuningStateToString(PidTuningState state) {
    switch (state) {
        case PidTuningState::IDLE:          return "IDLE";
        case PidTuningState::RUNNING:       return "RUNNING";
        case PidTuningState::PREVIEW_READY: return "PREVIEW_READY";
        case PidTuningState::SAVED:         return "SAVED";
        case PidTuningState::DISCARDED:     return "DISCARDED";
        case PidTuningState::FAILED:        return "FAILED";
        case PidTuningState::CANCELED:      return "CANCELED";
        default:                            return "UNKNOWN";
    }
}

static const char* pidTuningTargetToString(PidTuningTarget target) {
    switch (target) {
        case PidTuningTarget::MOTOR_SPEED_LEFT:  return "motor_speed_left";
        case PidTuningTarget::MOTOR_SPEED_RIGHT: return "motor_speed_right";
        default:                            return "unknown";
    }
}

static const char* pidTuningPhaseToString(PidTuningPhase phase) {
    switch (phase) {
        case PidTuningPhase::IDLE:                       return "IDLE";
        case PidTuningPhase::RESET:                      return "RESET";
        case PidTuningPhase::REST_BEFORE_STEP:           return "REST_BEFORE_STEP";
        case PidTuningPhase::LEFT_FORWARD:               return "LEFT_FORWARD";
        case PidTuningPhase::LEFT_REVERSE:               return "LEFT_REVERSE";
        case PidTuningPhase::RIGHT_FORWARD:              return "RIGHT_FORWARD";
        case PidTuningPhase::RIGHT_REVERSE:              return "RIGHT_REVERSE";
        case PidTuningPhase::COMPUTE:                    return "COMPUTE";
        case PidTuningPhase::VALIDATE_LEFT:              return "VALIDATE_LEFT";
        case PidTuningPhase::REST_BEFORE_VALIDATE_RIGHT: return "REST_BEFORE_VALIDATE_RIGHT";
        case PidTuningPhase::VALIDATE_RIGHT:             return "VALIDATE_RIGHT";
        case PidTuningPhase::PREVIEW:                    return "PREVIEW";
        default:                                         return "UNKNOWN";
    }
}

static const char* guidedCalibrationStateToString(GuidedCalibrationState state) {
    switch (state) {
        case GuidedCalibrationState::IDLE:     return "IDLE";
        case GuidedCalibrationState::RUNNING:  return "RUNNING";
        case GuidedCalibrationState::COMPLETE: return "COMPLETE";
        case GuidedCalibrationState::FAILED:   return "FAILED";
        case GuidedCalibrationState::CANCELED: return "CANCELED";
        default:                               return "UNKNOWN";
    }
}

static const char* guidedCalibrationPhaseToString(GuidedCalibrationPhase phase) {
    switch (phase) {
        case GuidedCalibrationPhase::IDLE:            return "IDLE";
        case GuidedCalibrationPhase::IMU_STILLNESS:   return "IMU_STILLNESS";
        case GuidedCalibrationPhase::LEFT_DIRECTION:  return "LEFT_DIRECTION";
        case GuidedCalibrationPhase::RIGHT_DIRECTION: return "RIGHT_DIRECTION";
        case GuidedCalibrationPhase::LEFT_DEADZONE:   return "LEFT_DEADZONE";
        case GuidedCalibrationPhase::RIGHT_DEADZONE:  return "RIGHT_DEADZONE";
        case GuidedCalibrationPhase::SUMMARY:         return "SUMMARY";
        default:                                      return "UNKNOWN";
    }
}

static cJSON* createPidJson(const PIDConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) return nullptr;
    cJSON_AddNumberToObject(obj, "kp", config.pid_kp);
    cJSON_AddNumberToObject(obj, "ki", config.pid_ki);
    cJSON_AddNumberToObject(obj, "kd", config.pid_kd);
    cJSON_AddNumberToObject(obj, "output_min", config.pid_output_min);
    cJSON_AddNumberToObject(obj, "output_max", config.pid_output_max);
    cJSON_AddNumberToObject(obj, "iterm_min", config.pid_iterm_min);
    cJSON_AddNumberToObject(obj, "iterm_max", config.pid_iterm_max);
    return obj;
}

static cJSON* createMetricsJson(const PidTuningResponseMetrics& metrics) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) return nullptr;
    cJSON_AddBoolToObject(obj, "valid", metrics.valid);
    cJSON_AddNumberToObject(obj, "forward_gain_dps_per_effort", metrics.forwardGain_dps_per_effort);
    cJSON_AddNumberToObject(obj, "reverse_gain_dps_per_effort", metrics.reverseGain_dps_per_effort);
    cJSON_AddNumberToObject(obj, "dead_time_s", metrics.deadTime_s);
    cJSON_AddNumberToObject(obj, "time_constant_s", metrics.timeConstant_s);
    cJSON_AddNumberToObject(obj, "steady_speed_forward_dps", metrics.steadySpeedForward_dps);
    cJSON_AddNumberToObject(obj, "steady_speed_reverse_dps", metrics.steadySpeedReverse_dps);
    return obj;
}

esp_err_t StateApiHandler::handleRequest(httpd_req_t *req) {
    ESP_LOGD(TAG, "Received request for /api/state (GET)");
    const SystemStatusSnapshot systemStatus = m_stateManager.getStatusSnapshot();
    const ConfigData configData = m_configService.getConfigData();
    const BatteryStatus batteryStatus = m_batteryService.getLatestStatus();
    const PidTuningStatus pidTuningStatus = m_pidTuningService.getStatus();
    const GuidedCalibrationStatus guidedStatus = m_guidedCalibrationService.getStatus();
    const OTAStatus otaStatus = m_otaService.getStatus();

    auto cjson_deleter = [](cJSON* ptr){ if(ptr) cJSON_Delete(ptr); };
    auto char_deleter = [](char* ptr){ if(ptr) free(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root_ptr(cJSON_CreateObject());
    cJSON* root = root_ptr.get();
    if (!root) {
        return sendHttp500(req);
    }

    cJSON_AddNumberToObject(root, "state_id", systemStatus.stateId);
    cJSON_AddStringToObject(root, "state_name", systemStatus.stateName);
    cJSON_AddBoolToObject(root, "auto_balancing_enabled", systemStatus.autoBalancingEnabled);
    cJSON_AddBoolToObject(root, "fall_detection_enabled", systemStatus.fallDetectionEnabled);
    cJSON_AddBoolToObject(root, "critical_battery_motor_shutdown_enabled", systemStatus.criticalBatteryMotorShutdownEnabled);
    cJSON_AddBoolToObject(root, "yaw_control_enabled", configData.control.yaw_control_enabled);
    cJSON_AddNumberToObject(root, "battery_voltage", batteryStatus.voltage);
    cJSON_AddNumberToObject(root, "battery_adc_pin_voltage", batteryStatus.adcPinVoltage);
    cJSON_AddNumberToObject(root, "battery_percentage", batteryStatus.percentage);
    cJSON_AddBoolToObject(root, "battery_is_low", batteryStatus.isLow);
    cJSON_AddBoolToObject(root, "battery_is_critical", batteryStatus.isCritical);
    cJSON_AddBoolToObject(root, "battery_adc_calibrated", batteryStatus.adcCalibrated);

    const auto imu = m_imuService.getStatusSnapshot();
    cJSON* imuObj = cJSON_AddObjectToObject(root, "imu");
    if (imuObj) {
        cJSON_AddStringToObject(imuObj, "state", imu.state);
        cJSON_AddBoolToObject(imuObj, "ready", imu.ready);
        cJSON_AddBoolToObject(imuObj, "busy", imu.busy);
        cJSON_AddBoolToObject(imuObj, "configuration_pending", imu.configurationPending);
        cJSON_AddNumberToObject(imuObj, "generation", imu.generation);
        cJSON_AddNumberToObject(imuObj, "revision", imu.revision);
        if (imu.sampleTimestampUs > 0)
            cJSON_AddNumberToObject(imuObj, "sample_age_ms", (esp_timer_get_time() - imu.sampleTimestampUs) / 1000.0);
        else cJSON_AddNullToObject(imuObj, "sample_age_ms");
        cJSON_AddNumberToObject(imuObj, "last_error", imu.lastError);
        cJSON_AddNumberToObject(imuObj, "last_reason", static_cast<unsigned>(imu.lastReason));
        cJSON_AddNumberToObject(imuObj, "state_changed_us", imu.stateChangedUs);
        cJSON_AddNumberToObject(imuObj, "transport_errors", imu.transportErrors);
        cJSON_AddNumberToObject(imuObj, "fifo_resyncs", imu.fifoResyncs);
        cJSON_AddNumberToObject(imuObj, "sample_sequence", static_cast<double>(imu.sampleSequence));
        cJSON_AddNumberToObject(imuObj, "fifo_remaining_packets", imu.fifoRemainingPackets);
        cJSON_AddNumberToObject(imuObj, "gyro_clipping_resets", imu.gyroClippingResets);
        cJSON_AddNumberToObject(imuObj, "reconnect_attempts", imu.reconnectAttempts);
        cJSON_AddNumberToObject(imuObj, "reconnect_successes", imu.reconnectSuccesses);
        cJSON_AddNumberToObject(imuObj, "irq_fallbacks", imu.irqFallbacks);
        cJSON_AddNumberToObject(imuObj, "bus_frequency_hz", imu.busFrequencyHz);
        cJSON_AddNumberToObject(imuObj, "sample_rate_hz", imu.samplePeriodUs ? 1000000.0 / imu.samplePeriodUs : 0);
    }

    cJSON* otaObj = cJSON_AddObjectToObject(root, "ota");
    if (otaObj) {
        cJSON_AddBoolToObject(otaObj, "available", otaStatus.available);
        cJSON_AddBoolToObject(otaObj, "spiffs_available", otaStatus.spiffsAvailable);
        cJSON_AddBoolToObject(otaObj, "update_allowed", otaStatus.updateAllowed);
        cJSON_AddBoolToObject(otaObj, "update_in_progress", otaStatus.updateInProgress);
        cJSON_AddBoolToObject(otaObj, "reboot_required", otaStatus.rebootRequired);
        cJSON_AddNumberToObject(otaObj, "bytes_written", static_cast<double>(otaStatus.bytesWritten));
        cJSON_AddNumberToObject(otaObj, "expected_size", static_cast<double>(otaStatus.expectedSize));
        cJSON_AddNumberToObject(otaObj, "spiffs_partition_size", static_cast<double>(otaStatus.spiffsPartitionSize));
        cJSON_AddStringToObject(otaObj, "running_partition", otaStatus.runningPartition.c_str());
        cJSON_AddStringToObject(otaObj, "update_partition", otaStatus.updatePartition.c_str());
        cJSON_AddStringToObject(otaObj, "spiffs_partition", otaStatus.spiffsPartition.c_str());
        cJSON_AddStringToObject(otaObj, "app_version", otaStatus.appVersion.c_str());
        cJSON_AddStringToObject(otaObj, "active_target", otaStatus.activeTarget.c_str());
        cJSON_AddStringToObject(otaObj, "message", otaStatus.message.c_str());
    }

    cJSON* tuningObj = cJSON_AddObjectToObject(root, "pid_tuning");
    if (tuningObj) {
        cJSON_AddStringToObject(tuningObj, "state", pidTuningStateToString(pidTuningStatus.state));
        cJSON_AddStringToObject(tuningObj, "target", pidTuningTargetToString(pidTuningStatus.target));
        cJSON_AddStringToObject(tuningObj, "phase", pidTuningPhaseToString(pidTuningStatus.phase));
        cJSON_AddNumberToObject(tuningObj, "progress", pidTuningStatus.progress);
        cJSON_AddStringToObject(tuningObj, "message", pidTuningStatus.message.c_str());
        cJSON_AddBoolToObject(tuningObj, "has_candidate", pidTuningStatus.hasCandidate);

        cJSON* candidateObj = cJSON_AddObjectToObject(tuningObj, "candidate");
        if (candidateObj) {
            cJSON_AddItemToObject(candidateObj, "speed_left", createPidJson(pidTuningStatus.candidateLeft));
            cJSON_AddItemToObject(candidateObj, "speed_right", createPidJson(pidTuningStatus.candidateRight));
        }

        cJSON* metricsObj = cJSON_AddObjectToObject(tuningObj, "metrics");
        if (metricsObj) {
            cJSON_AddItemToObject(metricsObj, "left", createMetricsJson(pidTuningStatus.leftMetrics));
            cJSON_AddItemToObject(metricsObj, "right", createMetricsJson(pidTuningStatus.rightMetrics));
        }
    }

    cJSON* guidedObj = cJSON_AddObjectToObject(root, "guided_calibration");
    if (guidedObj) {
        cJSON_AddStringToObject(guidedObj, "state", guidedCalibrationStateToString(guidedStatus.state));
        cJSON_AddStringToObject(guidedObj, "phase", guidedCalibrationPhaseToString(guidedStatus.phase));
        cJSON_AddNumberToObject(guidedObj, "progress", guidedStatus.progress);
        cJSON_AddStringToObject(guidedObj, "message", guidedStatus.message.c_str());
        cJSON_AddBoolToObject(guidedObj, "left_direction_ok", guidedStatus.leftDirectionOk);
        cJSON_AddBoolToObject(guidedObj, "right_direction_ok", guidedStatus.rightDirectionOk);
        cJSON_AddNumberToObject(guidedObj, "left_deadzone_effort", guidedStatus.leftDeadzoneEffort);
        cJSON_AddNumberToObject(guidedObj, "right_deadzone_effort", guidedStatus.rightDeadzoneEffort);
    }

    std::unique_ptr<char, decltype(char_deleter)> json_str_ptr(cJSON_PrintUnformatted(root));
    char* json_string = json_str_ptr.get();
    if (!json_string) {
        return sendHttp500(req);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json_string);
}
