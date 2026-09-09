#include "JsonConfigSectionCodecs.hpp"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

namespace {
constexpr const char* TAG = "JsonConfigCodecs";

#define GET_JSON_STRING(obj, key, target) \
    do { \
        cJSON* item = cJSON_GetObjectItem((obj), (key)); \
        if (item && cJSON_IsString(item)) { \
            (target) = item->valuestring; \
        } \
    } while (0)

#define GET_JSON_NUMBER_INT(obj, key, target) \
    do { \
        cJSON* item = cJSON_GetObjectItem((obj), (key)); \
        if (item && cJSON_IsNumber(item)) { \
            (target) = item->valueint; \
        } else { \
            ESP_LOGW(TAG, "Missing/invalid number '%s'", (key)); \
        } \
    } while (0)

#define GET_JSON_NUMBER_INT_CAST(obj, key, target, type) \
    do { \
        cJSON* item = cJSON_GetObjectItem((obj), (key)); \
        if (item && cJSON_IsNumber(item)) { \
            (target) = static_cast<type>(item->valueint); \
        } else { \
            ESP_LOGW(TAG, "Missing/invalid number '%s'", (key)); \
        } \
    } while (0)

#define GET_JSON_NUMBER_DOUBLE(obj, key, target) \
    do { \
        cJSON* item = cJSON_GetObjectItem((obj), (key)); \
        if (item && cJSON_IsNumber(item)) { \
            (target) = item->valuedouble; \
        } else { \
            ESP_LOGW(TAG, "Missing/invalid number '%s'", (key)); \
        } \
    } while (0)

#define GET_JSON_BOOL(obj, key, target) \
    do { \
        cJSON* item = cJSON_GetObjectItem((obj), (key)); \
        if (item && cJSON_IsBool(item)) { \
            (target) = cJSON_IsTrue(item); \
        } else { \
            ESP_LOGW(TAG, "Missing/invalid bool '%s'", (key)); \
        } \
    } while (0)
}

namespace json_config_sections {

cJSON* serializeWiFi(const WiFiConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddStringToObject(obj, "ssid", config.ssid.c_str());
    cJSON_AddStringToObject(obj, "password", config.password.c_str());
    return obj;
}

bool deserializeWiFi(cJSON* obj, WiFiConfig& config) {
    if (!obj) {
        return false;
    }
    GET_JSON_STRING(obj, "ssid", config.ssid);
    GET_JSON_STRING(obj, "password", config.password);
    return true;
}

cJSON* serializeMainLoop(const MainLoopConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "interval_ms", config.interval_ms);
    return obj;
}

bool deserializeMainLoop(cJSON* obj, MainLoopConfig& config) {
    if (!obj) {
        return false;
    }
    GET_JSON_NUMBER_INT(obj, "interval_ms", config.interval_ms);
    return true;
}

cJSON* serializeControl(const ControlConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "joystick_exponent", config.joystick_exponent);
    cJSON_AddNumberToObject(obj, "max_target_pitch_offset_deg",
                            config.strategies.nested_pid.max_target_pitch_offset_deg);
    cJSON_AddBoolToObject(obj, "yaw_control_enabled",
                          config.strategies.nested_pid.yaw_control_enabled);
    cJSON_AddStringToObject(obj, "balance_strategy",
                            balanceStrategyIdToString(config.strategies.active));
    cJSON_AddNumberToObject(obj, "strategies_revision", config.strategies.revision);
    cJSON* strategies = serializeBalanceStrategies(config.strategies);
    if (!strategies || !cJSON_AddItemToObject(obj, "strategies", strategies)) {
        if (strategies) cJSON_Delete(strategies);
        cJSON_Delete(obj);
        return nullptr;
    }
    return obj;
}

bool deserializeControl(cJSON* obj, ControlConfig& config) {
    bool ok = true;
    cJSON* item = cJSON_GetObjectItem(obj, "joystick_exponent");
    if (item && cJSON_IsNumber(item)) {
        config.joystick_exponent = item->valuedouble;
    } else {
        ok = false;
        ESP_LOGW(TAG, "Missing/invalid 'joystick_exponent'");
    }
    item = cJSON_GetObjectItem(obj, "max_target_pitch_offset_deg");
    const bool hasLegacyMaxPitch = item && cJSON_IsNumber(item);
    if (hasLegacyMaxPitch) config.max_target_pitch_offset_deg = item->valuedouble;
    item = cJSON_GetObjectItem(obj, "yaw_control_enabled");
    const bool hasLegacyYaw = item && cJSON_IsBool(item);
    if (hasLegacyYaw) config.yaw_control_enabled = cJSON_IsTrue(item);
    cJSON* strategies = cJSON_GetObjectItem(obj, "strategies");
    if (strategies) {
        if (!deserializeBalanceStrategies(strategies, config.strategies)) ok = false;
        // These fields are retained for old UI clients. When present, treat
        // them as a compatibility patch and copy them into the canonical
        // NestedPid record before validation.
        if (hasLegacyMaxPitch) {
            config.strategies.nested_pid.max_target_pitch_offset_deg = config.max_target_pitch_offset_deg;
        }
        if (hasLegacyYaw) {
            config.strategies.nested_pid.yaw_control_enabled = config.yaw_control_enabled;
        }
        config.max_target_pitch_offset_deg = config.strategies.nested_pid.max_target_pitch_offset_deg;
        config.yaw_control_enabled = config.strategies.nested_pid.yaw_control_enabled;
    }
    cJSON* strategyItem = cJSON_GetObjectItem(obj, "balance_strategy");
    if (strategyItem && cJSON_IsString(strategyItem)) {
        BalanceStrategyId requestedStrategy = config.strategies.active;
        if (!balanceStrategyIdFromString(strategyItem->valuestring, requestedStrategy)) {
            ok = false;
            ESP_LOGW(TAG, "Unknown 'balance_strategy': %s", strategyItem->valuestring);
        } else if (strategies && requestedStrategy != config.strategies.active) {
            ok = false;
            ESP_LOGW(TAG, "Conflicting active strategy identifiers");
        } else {
            config.strategies.active = requestedStrategy;
        }
    } else if (strategies) {
        ok = false;
        ESP_LOGW(TAG, "Missing/invalid 'balance_strategy'");
    }
    cJSON* revisionItem = cJSON_GetObjectItem(obj, "strategies_revision");
    if (revisionItem && cJSON_IsNumber(revisionItem) && revisionItem->valueint >= 0) {
        config.strategies.revision = static_cast<uint32_t>(revisionItem->valueint);
    }
    if (!ok) {
        ESP_LOGW(TAG, "Error(s) parsing Control config.");
    }
    return ok;
}

cJSON* serializeBalanceStrategies(const BalanceStrategiesConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) return nullptr;
    cJSON_AddStringToObject(obj, "active", balanceStrategyIdToString(config.active));
    cJSON_AddNumberToObject(obj, "revision", config.revision);

    cJSON* nested = cJSON_CreateObject();
    cJSON* longitudinal = cJSON_CreateObject();
    if (!nested || !longitudinal) {
        if (nested) cJSON_Delete(nested);
        if (longitudinal) cJSON_Delete(longitudinal);
        cJSON_Delete(obj);
        return nullptr;
    }

    const auto addPid = [](cJSON* parent, const char* name, const PIDConfig& pid) {
        cJSON* section = serializePid(pid);
        return section && cJSON_AddItemToObject(parent, name, section);
    };
    if (!addPid(nested, "angle", config.nested_pid.angle) ||
        !addPid(nested, "speed_left", config.nested_pid.speed_left) ||
        !addPid(nested, "speed_right", config.nested_pid.speed_right) ||
        !addPid(nested, "yaw_angle", config.nested_pid.yaw_angle) ||
        !addPid(nested, "yaw_rate", config.nested_pid.yaw_rate)) {
        cJSON_Delete(nested);
        cJSON_Delete(longitudinal);
        cJSON_Delete(obj);
        return nullptr;
    }
    cJSON_AddNumberToObject(nested, "max_target_pitch_offset_deg",
                            config.nested_pid.max_target_pitch_offset_deg);
    cJSON_AddBoolToObject(nested, "yaw_control_enabled",
                          config.nested_pid.yaw_control_enabled);
    cJSON_AddNumberToObject(nested, "revision", config.nested_pid.revision);

    const auto addNumber = [](cJSON* parent, const char* name, float value) {
        return cJSON_AddNumberToObject(parent, name, value) != nullptr;
    };
    if (!addPid(longitudinal, "pitch", config.longitudinal_cascade.pitch) ||
        !addPid(longitudinal, "velocity", config.longitudinal_cascade.velocity) ||
        !addNumber(longitudinal, "position_kp", config.longitudinal_cascade.position_kp) ||
        !addNumber(longitudinal, "pitch_trim_deg", config.longitudinal_cascade.pitch_trim_deg) ||
        !addNumber(longitudinal, "max_pitch_offset_deg", config.longitudinal_cascade.max_pitch_offset_deg) ||
        !addNumber(longitudinal, "max_pitch_rate_dps", config.longitudinal_cascade.max_pitch_rate_dps) ||
        !addNumber(longitudinal, "max_velocity_mps", config.longitudinal_cascade.max_velocity_mps) ||
        !addNumber(longitudinal, "max_hold_velocity_mps", config.longitudinal_cascade.max_hold_velocity_mps) ||
        !addNumber(longitudinal, "max_acceleration_mps2", config.longitudinal_cascade.max_acceleration_mps2) ||
        !addNumber(longitudinal, "max_deceleration_mps2", config.longitudinal_cascade.max_deceleration_mps2) ||
        !addNumber(longitudinal, "hold_position_deadband_m", config.longitudinal_cascade.hold_position_deadband_m) ||
        !addNumber(longitudinal, "hold_velocity_deadband_mps", config.longitudinal_cascade.hold_velocity_deadband_mps) ||
        !addNumber(longitudinal, "sync_kp", config.longitudinal_cascade.sync_kp) ||
        !addNumber(longitudinal, "sync_kd", config.longitudinal_cascade.sync_kd) ||
        !addNumber(longitudinal, "sync_max_effort", config.longitudinal_cascade.sync_max_effort) ||
        !addNumber(longitudinal, "max_effort", config.longitudinal_cascade.max_effort)) {
        cJSON_Delete(nested);
        cJSON_Delete(longitudinal);
        cJSON_Delete(obj);
        return nullptr;
    }
    cJSON_AddBoolToObject(longitudinal, "configured", config.longitudinal_cascade.configured);
    cJSON_AddNumberToObject(longitudinal, "revision", config.longitudinal_cascade.revision);

    if (!cJSON_AddItemToObject(obj, "nested_pid", nested) ||
        !cJSON_AddItemToObject(obj, "longitudinal_cascade", longitudinal)) {
        cJSON_Delete(obj);
        return nullptr;
    }
    return obj;
}

bool deserializeBalanceStrategies(cJSON* obj, BalanceStrategiesConfig& config) {
    if (!obj || !cJSON_IsObject(obj)) return false;
    bool ok = true;
    auto parseNumber = [&](cJSON* parent, const char* name, float& target, bool required) {
        cJSON* item = cJSON_GetObjectItem(parent, name);
        if (item && cJSON_IsNumber(item)) {
            target = static_cast<float>(item->valuedouble);
        } else if (required) {
            ok = false;
        }
    };
    auto parseRevision = [&](cJSON* parent, const char* name, uint32_t& target) {
        cJSON* item = cJSON_GetObjectItem(parent, name);
        if (item && cJSON_IsNumber(item) && item->valueint >= 0) {
            target = static_cast<uint32_t>(item->valueint);
        }
    };
    cJSON* nested = cJSON_GetObjectItem(obj, "nested_pid");
    cJSON* longitudinal = cJSON_GetObjectItem(obj, "longitudinal_cascade");
    if (!nested || !cJSON_IsObject(nested) || !longitudinal || !cJSON_IsObject(longitudinal)) {
        ESP_LOGW(TAG, "Both strategy configuration sections are required");
        return false;
    }
    if (!deserializePid(cJSON_GetObjectItem(nested, "angle"), config.nested_pid.angle) ||
        !deserializePid(cJSON_GetObjectItem(nested, "speed_left"), config.nested_pid.speed_left) ||
        !deserializePid(cJSON_GetObjectItem(nested, "speed_right"), config.nested_pid.speed_right) ||
        !deserializePid(cJSON_GetObjectItem(nested, "yaw_angle"), config.nested_pid.yaw_angle) ||
        !deserializePid(cJSON_GetObjectItem(nested, "yaw_rate"), config.nested_pid.yaw_rate)) {
        ok = false;
    }
    parseNumber(nested, "max_target_pitch_offset_deg",
                config.nested_pid.max_target_pitch_offset_deg, true);
    cJSON* yaw = cJSON_GetObjectItem(nested, "yaw_control_enabled");
    if (yaw && cJSON_IsBool(yaw)) config.nested_pid.yaw_control_enabled = cJSON_IsTrue(yaw);
    else ok = false;
    parseRevision(nested, "revision", config.nested_pid.revision);

    if (!deserializePid(cJSON_GetObjectItem(longitudinal, "pitch"), config.longitudinal_cascade.pitch) ||
        !deserializePid(cJSON_GetObjectItem(longitudinal, "velocity"), config.longitudinal_cascade.velocity)) {
        ok = false;
    }
    parseNumber(longitudinal, "position_kp", config.longitudinal_cascade.position_kp, true);
    parseNumber(longitudinal, "pitch_trim_deg", config.longitudinal_cascade.pitch_trim_deg, true);
    parseNumber(longitudinal, "max_pitch_offset_deg", config.longitudinal_cascade.max_pitch_offset_deg, true);
    parseNumber(longitudinal, "max_pitch_rate_dps", config.longitudinal_cascade.max_pitch_rate_dps, true);
    parseNumber(longitudinal, "max_velocity_mps", config.longitudinal_cascade.max_velocity_mps, true);
    parseNumber(longitudinal, "max_hold_velocity_mps", config.longitudinal_cascade.max_hold_velocity_mps, true);
    parseNumber(longitudinal, "max_acceleration_mps2", config.longitudinal_cascade.max_acceleration_mps2, true);
    parseNumber(longitudinal, "max_deceleration_mps2", config.longitudinal_cascade.max_deceleration_mps2, true);
    parseNumber(longitudinal, "hold_position_deadband_m", config.longitudinal_cascade.hold_position_deadband_m, true);
    parseNumber(longitudinal, "hold_velocity_deadband_mps", config.longitudinal_cascade.hold_velocity_deadband_mps, true);
    parseNumber(longitudinal, "sync_kp", config.longitudinal_cascade.sync_kp, true);
    parseNumber(longitudinal, "sync_kd", config.longitudinal_cascade.sync_kd, true);
    parseNumber(longitudinal, "sync_max_effort", config.longitudinal_cascade.sync_max_effort, true);
    parseNumber(longitudinal, "max_effort", config.longitudinal_cascade.max_effort, true);
    cJSON* configured = cJSON_GetObjectItem(longitudinal, "configured");
    if (configured && cJSON_IsBool(configured)) config.longitudinal_cascade.configured = cJSON_IsTrue(configured);
    else ok = false;
    parseRevision(longitudinal, "revision", config.longitudinal_cascade.revision);

    cJSON* active = cJSON_GetObjectItem(obj, "active");
    if (active && cJSON_IsString(active) &&
        balanceStrategyIdFromString(active->valuestring, config.active)) {
        // Parsed successfully.
    } else {
        ok = false;
    }
    parseRevision(obj, "revision", config.revision);
    return ok;
}

cJSON* serializeImu(const MPU6050Config& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "i2c_port", config.i2c_port);
    cJSON_AddNumberToObject(obj, "sda_pin", config.sda_pin);
    cJSON_AddNumberToObject(obj, "scl_pin", config.scl_pin);
    cJSON_AddNumberToObject(obj, "device_address", config.device_address);
    cJSON_AddNumberToObject(obj, "i2c_freq_hz", config.i2c_freq_hz);
    cJSON_AddNumberToObject(obj, "int_pin", config.int_pin);
    cJSON_AddBoolToObject(obj, "interrupt_active_high", config.interrupt_active_high);
    cJSON_AddNumberToObject(obj, "accel_range", config.accel_range);
    cJSON_AddNumberToObject(obj, "gyro_range", config.gyro_range);
    cJSON_AddNumberToObject(obj, "dlpf_config", config.dlpf_config);
    cJSON_AddNumberToObject(obj, "sample_rate_divisor", config.sample_rate_divisor);
    cJSON_AddNumberToObject(obj, "calibration_samples", config.calibration_samples);
    cJSON_AddNumberToObject(obj, "comp_filter_alpha", config.comp_filter_alpha);
    cJSON_AddNumberToObject(obj, "fifo_read_threshold", config.fifo_read_threshold);
    cJSON_AddNumberToObject(obj, "gyro_offset_x", config.gyro_offset_x);
    cJSON_AddNumberToObject(obj, "gyro_offset_y", config.gyro_offset_y);
    cJSON_AddNumberToObject(obj, "gyro_offset_z", config.gyro_offset_z);
    return obj;
}

bool deserializeImu(cJSON* obj, MPU6050Config& config) {
    if (!obj) {
        return false;
    }
    GET_JSON_NUMBER_INT_CAST(obj, "i2c_port", config.i2c_port, i2c_port_t);
    GET_JSON_NUMBER_INT_CAST(obj, "sda_pin", config.sda_pin, gpio_num_t);
    GET_JSON_NUMBER_INT_CAST(obj, "scl_pin", config.scl_pin, gpio_num_t);
    GET_JSON_NUMBER_INT(obj, "device_address", config.device_address);
    GET_JSON_NUMBER_INT(obj, "i2c_freq_hz", config.i2c_freq_hz);
    GET_JSON_NUMBER_INT_CAST(obj, "int_pin", config.int_pin, gpio_num_t);
    GET_JSON_BOOL(obj, "interrupt_active_high", config.interrupt_active_high);
    GET_JSON_NUMBER_INT(obj, "accel_range", config.accel_range);
    GET_JSON_NUMBER_INT(obj, "gyro_range", config.gyro_range);
    GET_JSON_NUMBER_INT(obj, "dlpf_config", config.dlpf_config);
    GET_JSON_NUMBER_INT(obj, "sample_rate_divisor", config.sample_rate_divisor);
    GET_JSON_NUMBER_INT(obj, "calibration_samples", config.calibration_samples);
    GET_JSON_NUMBER_DOUBLE(obj, "comp_filter_alpha", config.comp_filter_alpha);
    GET_JSON_NUMBER_INT(obj, "fifo_read_threshold", config.fifo_read_threshold);
    GET_JSON_NUMBER_DOUBLE(obj, "gyro_offset_x", config.gyro_offset_x);
    GET_JSON_NUMBER_DOUBLE(obj, "gyro_offset_y", config.gyro_offset_y);
    GET_JSON_NUMBER_DOUBLE(obj, "gyro_offset_z", config.gyro_offset_z);
    return true;
}

cJSON* serializeEncoder(const EncoderConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "left_pin_a", config.left_pin_a);
    cJSON_AddNumberToObject(obj, "left_pin_b", config.left_pin_b);
    cJSON_AddNumberToObject(obj, "right_pin_a", config.right_pin_a);
    cJSON_AddNumberToObject(obj, "right_pin_b", config.right_pin_b);
    cJSON_AddNumberToObject(obj, "pcnt_high_limit", config.pcnt_high_limit);
    cJSON_AddNumberToObject(obj, "pcnt_low_limit", config.pcnt_low_limit);
    cJSON_AddNumberToObject(obj, "pcnt_filter_ns", config.pcnt_filter_ns);
    cJSON_AddNumberToObject(obj, "pulses_per_revolution_motor", config.pulses_per_revolution_motor);
    cJSON_AddNumberToObject(obj, "gear_ratio", config.gear_ratio);
    cJSON_AddNumberToObject(obj, "wheel_diameter_mm", config.wheel_diameter_mm);
    cJSON_AddNumberToObject(obj, "speed_filter_alpha", config.speed_filter_alpha);
    return obj;
}

bool deserializeEncoder(cJSON* obj, EncoderConfig& config) {
    if (!obj) {
        return false;
    }
    GET_JSON_NUMBER_INT_CAST(obj, "left_pin_a", config.left_pin_a, gpio_num_t);
    GET_JSON_NUMBER_INT_CAST(obj, "left_pin_b", config.left_pin_b, gpio_num_t);
    GET_JSON_NUMBER_INT_CAST(obj, "right_pin_a", config.right_pin_a, gpio_num_t);
    GET_JSON_NUMBER_INT_CAST(obj, "right_pin_b", config.right_pin_b, gpio_num_t);
    GET_JSON_NUMBER_INT(obj, "pcnt_high_limit", config.pcnt_high_limit);
    GET_JSON_NUMBER_INT(obj, "pcnt_low_limit", config.pcnt_low_limit);
    GET_JSON_NUMBER_INT(obj, "pcnt_filter_ns", config.pcnt_filter_ns);
    GET_JSON_NUMBER_DOUBLE(obj, "pulses_per_revolution_motor", config.pulses_per_revolution_motor);
    GET_JSON_NUMBER_DOUBLE(obj, "gear_ratio", config.gear_ratio);
    GET_JSON_NUMBER_DOUBLE(obj, "wheel_diameter_mm", config.wheel_diameter_mm);
    GET_JSON_NUMBER_DOUBLE(obj, "speed_filter_alpha", config.speed_filter_alpha);
    return true;
}

cJSON* serializeMotor(const MotorConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "left_pin_in1", config.left_pin_in1);
    cJSON_AddNumberToObject(obj, "left_pin_in2", config.left_pin_in2);
    cJSON_AddNumberToObject(obj, "left_channel_1", config.left_channel_1);
    cJSON_AddNumberToObject(obj, "left_channel_2", config.left_channel_2);
    cJSON_AddNumberToObject(obj, "right_pin_in1", config.right_pin_in1);
    cJSON_AddNumberToObject(obj, "right_pin_in2", config.right_pin_in2);
    cJSON_AddNumberToObject(obj, "right_channel_1", config.right_channel_1);
    cJSON_AddNumberToObject(obj, "right_channel_2", config.right_channel_2);
    cJSON_AddNumberToObject(obj, "timer_num", config.timer_num);
    cJSON_AddNumberToObject(obj, "speed_mode", config.speed_mode);
    cJSON_AddNumberToObject(obj, "pwm_frequency_hz", config.pwm_frequency_hz);
    cJSON_AddNumberToObject(obj, "duty_resolution", config.duty_resolution);
    cJSON_AddNumberToObject(obj, "deadzone_duty", config.deadzone_duty);
    return obj;
}

bool deserializeMotor(cJSON* obj, MotorConfig& config) {
    if (!obj) {
        return false;
    }
    GET_JSON_NUMBER_INT_CAST(obj, "left_pin_in1", config.left_pin_in1, gpio_num_t);
    GET_JSON_NUMBER_INT_CAST(obj, "left_pin_in2", config.left_pin_in2, gpio_num_t);
    GET_JSON_NUMBER_INT_CAST(obj, "left_channel_1", config.left_channel_1, ledc_channel_t);
    GET_JSON_NUMBER_INT_CAST(obj, "left_channel_2", config.left_channel_2, ledc_channel_t);
    GET_JSON_NUMBER_INT_CAST(obj, "right_pin_in1", config.right_pin_in1, gpio_num_t);
    GET_JSON_NUMBER_INT_CAST(obj, "right_pin_in2", config.right_pin_in2, gpio_num_t);
    GET_JSON_NUMBER_INT_CAST(obj, "right_channel_1", config.right_channel_1, ledc_channel_t);
    GET_JSON_NUMBER_INT_CAST(obj, "right_channel_2", config.right_channel_2, ledc_channel_t);
    GET_JSON_NUMBER_INT_CAST(obj, "timer_num", config.timer_num, ledc_timer_t);
    GET_JSON_NUMBER_INT_CAST(obj, "speed_mode", config.speed_mode, ledc_mode_t);
    GET_JSON_NUMBER_INT(obj, "pwm_frequency_hz", config.pwm_frequency_hz);
    GET_JSON_NUMBER_INT_CAST(obj, "duty_resolution", config.duty_resolution, ledc_timer_bit_t);
    GET_JSON_NUMBER_INT(obj, "deadzone_duty", config.deadzone_duty);
    return true;
}

cJSON* serializeBattery(const BatteryConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "adc_pin", config.adc_pin);
    cJSON_AddNumberToObject(obj, "voltage_divider_ratio", config.voltage_divider_ratio);
    cJSON_AddNumberToObject(obj, "voltage_max", config.voltage_max);
    cJSON_AddNumberToObject(obj, "voltage_min", config.voltage_min);
    cJSON_AddBoolToObject(obj, "critical_battery_motor_shutdown_enabled", config.critical_battery_motor_shutdown_enabled);
    cJSON_AddNumberToObject(obj, "adc_bitwidth", config.adc_bitwidth);
    cJSON_AddNumberToObject(obj, "adc_atten", config.adc_atten);
    return obj;
}

bool deserializeBattery(cJSON* obj, BatteryConfig& config) {
    if (!obj) {
        return false;
    }
    GET_JSON_NUMBER_INT_CAST(obj, "adc_pin", config.adc_pin, gpio_num_t);
    GET_JSON_NUMBER_DOUBLE(obj, "voltage_divider_ratio", config.voltage_divider_ratio);
    GET_JSON_NUMBER_DOUBLE(obj, "voltage_max", config.voltage_max);
    GET_JSON_NUMBER_DOUBLE(obj, "voltage_min", config.voltage_min);
    GET_JSON_BOOL(obj, "critical_battery_motor_shutdown_enabled", config.critical_battery_motor_shutdown_enabled);
    GET_JSON_NUMBER_INT_CAST(obj, "adc_bitwidth", config.adc_bitwidth, adc_bitwidth_t);
    GET_JSON_NUMBER_INT_CAST(obj, "adc_atten", config.adc_atten, adc_atten_t);
    return true;
}

cJSON* serializeBehavior(const SystemBehaviorConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "joystick_deadzone", config.joystick_deadzone);
    cJSON_AddNumberToObject(obj, "joystick_timeout_ms", config.joystick_timeout_ms);
    cJSON_AddNumberToObject(obj, "joystick_check_interval_ms", config.joystick_check_interval_ms);
    cJSON_AddNumberToObject(obj, "max_target_angular_velocity_dps", config.max_target_angular_velocity_dps);
    cJSON_AddNumberToObject(obj, "fall_pitch_threshold_deg", config.fall_pitch_threshold_deg);
    cJSON_AddNumberToObject(obj, "fall_threshold_duration_ms", config.fall_threshold_duration_ms);
    cJSON_AddNumberToObject(obj, "auto_balance_pitch_threshold_deg", config.auto_balance_pitch_threshold_deg);
    cJSON_AddNumberToObject(obj, "auto_balance_hold_duration_ms", config.auto_balance_hold_duration_ms);
    cJSON_AddNumberToObject(obj, "battery_oversampling_count", config.battery_oversampling_count);
    cJSON_AddNumberToObject(obj, "battery_read_interval_ms", config.battery_read_interval_ms);
    cJSON_AddNumberToObject(obj, "imu_reconnect_interval_ms", config.imu_reconnect_interval_ms);
    cJSON_AddNumberToObject(obj, "imu_max_sample_age_ms", config.imu_max_sample_age_ms);
    return obj;
}

bool deserializeBehavior(cJSON* obj, SystemBehaviorConfig& config) {
    if (!obj) {
        return false;
    }
    GET_JSON_NUMBER_DOUBLE(obj, "joystick_deadzone", config.joystick_deadzone);
    GET_JSON_NUMBER_INT(obj, "joystick_timeout_ms", config.joystick_timeout_ms);
    GET_JSON_NUMBER_INT(obj, "joystick_check_interval_ms", config.joystick_check_interval_ms);
    GET_JSON_NUMBER_DOUBLE(obj, "max_target_angular_velocity_dps", config.max_target_angular_velocity_dps);
    GET_JSON_NUMBER_DOUBLE(obj, "fall_pitch_threshold_deg", config.fall_pitch_threshold_deg);
    GET_JSON_NUMBER_INT(obj, "fall_threshold_duration_ms", config.fall_threshold_duration_ms);

    cJSON* auto_balance_pitch_item = cJSON_GetObjectItem(obj, "auto_balance_pitch_threshold_deg");
    if (auto_balance_pitch_item && cJSON_IsNumber(auto_balance_pitch_item)) {
        config.auto_balance_pitch_threshold_deg = auto_balance_pitch_item->valuedouble;
    } else {
        GET_JSON_NUMBER_DOUBLE(obj, "recovery_pitch_threshold_deg", config.auto_balance_pitch_threshold_deg);
    }

    cJSON* auto_balance_hold_item = cJSON_GetObjectItem(obj, "auto_balance_hold_duration_ms");
    if (auto_balance_hold_item && cJSON_IsNumber(auto_balance_hold_item)) {
        config.auto_balance_hold_duration_ms = auto_balance_hold_item->valueint;
    } else {
        GET_JSON_NUMBER_INT(obj, "recovery_hold_duration_ms", config.auto_balance_hold_duration_ms);
    }

    GET_JSON_NUMBER_INT(obj, "battery_oversampling_count", config.battery_oversampling_count);
    GET_JSON_NUMBER_INT(obj, "battery_read_interval_ms", config.battery_read_interval_ms);
    if (const auto item = cJSON_GetObjectItem(obj, "imu_reconnect_interval_ms")) {
        if (!cJSON_IsNumber(item) || item->valuedouble != item->valueint) return false;
        config.imu_reconnect_interval_ms = item->valueint;
    }
    if (const auto item = cJSON_GetObjectItem(obj, "imu_max_sample_age_ms")) {
        if (!cJSON_IsNumber(item) || item->valuedouble != item->valueint) return false;
        config.imu_max_sample_age_ms = item->valueint;
    }
    return true;
}

cJSON* serializeDimensions(const RobotDimensionsConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "wheelbase_m", config.wheelbase_m);
    return obj;
}

bool deserializeDimensions(cJSON* obj, RobotDimensionsConfig& config) {
    if (!obj) {
        return false;
    }
    GET_JSON_NUMBER_DOUBLE(obj, "wheelbase_m", config.wheelbase_m);
    return true;
}

cJSON* serializeWeb(const WebServerConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "telemetry_buffer_size", config.telemetry_buffer_size);
    cJSON_AddNumberToObject(obj, "max_config_post_size", config.max_config_post_size);
    cJSON_AddBoolToObject(obj, "web_logs_enabled", config.web_logs_enabled);
    cJSON_AddNumberToObject(obj, "log_buffer_lines", config.log_buffer_lines);
    cJSON_AddNumberToObject(obj, "log_line_max_length", config.log_line_max_length);
    return obj;
}

bool deserializeWeb(cJSON* obj, WebServerConfig& config) {
    if (!obj) {
        return false;
    }
    GET_JSON_NUMBER_INT(obj, "telemetry_buffer_size", config.telemetry_buffer_size);
    GET_JSON_NUMBER_INT(obj, "max_config_post_size", config.max_config_post_size);
    GET_JSON_BOOL(obj, "web_logs_enabled", config.web_logs_enabled);
    GET_JSON_NUMBER_INT(obj, "log_buffer_lines", config.log_buffer_lines);
    GET_JSON_NUMBER_INT(obj, "log_line_max_length", config.log_line_max_length);
    return true;
}

cJSON* serializePid(const PIDConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "kp", config.pid_kp);
    cJSON_AddNumberToObject(obj, "ki", config.pid_ki);
    cJSON_AddNumberToObject(obj, "kd", config.pid_kd);
    cJSON_AddNumberToObject(obj, "output_min", config.pid_output_min);
    cJSON_AddNumberToObject(obj, "output_max", config.pid_output_max);
    cJSON_AddNumberToObject(obj, "iterm_min", config.pid_iterm_min);
    cJSON_AddNumberToObject(obj, "iterm_max", config.pid_iterm_max);
    return obj;
}

bool deserializePid(cJSON* obj, PIDConfig& config) {
    bool ok = true;
    cJSON* item = nullptr;

#define PARSE_PID_FIELD(key, target) \
    item = cJSON_GetObjectItem(obj, (key)); \
    if (item && cJSON_IsNumber(item)) { \
        (target) = item->valuedouble; \
    } else { \
        ok = false; \
    }

    PARSE_PID_FIELD("kp", config.pid_kp);
    PARSE_PID_FIELD("ki", config.pid_ki);
    PARSE_PID_FIELD("kd", config.pid_kd);
    PARSE_PID_FIELD("output_min", config.pid_output_min);
    PARSE_PID_FIELD("output_max", config.pid_output_max);
    PARSE_PID_FIELD("iterm_min", config.pid_iterm_min);
    PARSE_PID_FIELD("iterm_max", config.pid_iterm_max);

#undef PARSE_PID_FIELD

    if (!ok) {
        ESP_LOGW(TAG, "Missing or invalid field in PID config section.");
    }
    return ok;
}

cJSON* serializePidTuning(const PidTuningConfig& config) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) {
        return nullptr;
    }
    cJSON_AddNumberToObject(obj, "step_effort", config.step_effort);
    cJSON_AddNumberToObject(obj, "max_effort", config.max_effort);
    cJSON_AddNumberToObject(obj, "step_duration_ms", config.step_duration_ms);
    cJSON_AddNumberToObject(obj, "rest_duration_ms", config.rest_duration_ms);
    cJSON_AddNumberToObject(obj, "min_response_dps", config.min_response_dps);
    cJSON_AddNumberToObject(obj, "max_speed_dps", config.max_speed_dps);
    cJSON_AddNumberToObject(obj, "validation_target_dps", config.validation_target_dps);
    cJSON_AddNumberToObject(obj, "gain_scale", config.gain_scale);
    return obj;
}

bool deserializePidTuning(cJSON* obj, PidTuningConfig& config) {
    if (!obj) {
        return false;
    }
    GET_JSON_NUMBER_DOUBLE(obj, "step_effort", config.step_effort);
    GET_JSON_NUMBER_DOUBLE(obj, "max_effort", config.max_effort);
    GET_JSON_NUMBER_INT(obj, "step_duration_ms", config.step_duration_ms);
    GET_JSON_NUMBER_INT(obj, "rest_duration_ms", config.rest_duration_ms);
    GET_JSON_NUMBER_DOUBLE(obj, "min_response_dps", config.min_response_dps);
    GET_JSON_NUMBER_DOUBLE(obj, "max_speed_dps", config.max_speed_dps);
    GET_JSON_NUMBER_DOUBLE(obj, "validation_target_dps", config.validation_target_dps);
    GET_JSON_NUMBER_DOUBLE(obj, "gain_scale", config.gain_scale);
    return true;
}

}
