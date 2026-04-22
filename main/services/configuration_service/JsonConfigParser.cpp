#include "JsonConfigParser.hpp"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_err.h"
#include <string>
#include <vector>
#include <memory>
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"

// #include "ConfigData.hpp" // Included via header

// --- Fully implement helper functions ---

cJSON* JsonConfigParser::serializeBehavior(const SystemBehaviorConfig& b) const {
    cJSON *obj = cJSON_CreateObject(); if (!obj) return nullptr;
    cJSON_AddNumberToObject(obj, "joystick_deadzone", b.joystick_deadzone);
    cJSON_AddNumberToObject(obj, "joystick_timeout_ms", b.joystick_timeout_ms);
    cJSON_AddNumberToObject(obj, "joystick_check_interval_ms", b.joystick_check_interval_ms);
    cJSON_AddNumberToObject(obj, "max_target_angular_velocity_dps", b.max_target_angular_velocity_dps);
    cJSON_AddNumberToObject(obj, "fall_pitch_threshold_deg", b.fall_pitch_threshold_deg);
    cJSON_AddNumberToObject(obj, "fall_threshold_duration_ms", b.fall_threshold_duration_ms);
    cJSON_AddNumberToObject(obj, "recovery_pitch_threshold_deg", b.recovery_pitch_threshold_deg);
    cJSON_AddNumberToObject(obj, "recovery_hold_duration_ms", b.recovery_hold_duration_ms);
    cJSON_AddNumberToObject(obj, "battery_oversampling_count", b.battery_oversampling_count);
    cJSON_AddNumberToObject(obj, "battery_read_interval_ms", b.battery_read_interval_ms);
    cJSON_AddNumberToObject(obj, "imu_health_i2c_fail_threshold", b.imu_health_i2c_fail_threshold);
    cJSON_AddNumberToObject(obj, "imu_health_no_data_threshold", b.imu_health_no_data_threshold);
    cJSON_AddNumberToObject(obj, "imu_health_data_timeout_ms", b.imu_health_data_timeout_ms);
    return obj;
}

bool JsonConfigParser::deserializeBehavior(cJSON* obj, SystemBehaviorConfig& b) const {
    if (!obj) return false;
    GET_JSON_NUMBER_DOUBLE(obj, "joystick_deadzone", b.joystick_deadzone);
    GET_JSON_NUMBER_INT(obj, "joystick_timeout_ms", b.joystick_timeout_ms);
    GET_JSON_NUMBER_INT(obj, "joystick_check_interval_ms", b.joystick_check_interval_ms);
    GET_JSON_NUMBER_DOUBLE(obj, "max_target_angular_velocity_dps", b.max_target_angular_velocity_dps);
    GET_JSON_NUMBER_DOUBLE(obj, "fall_pitch_threshold_deg", b.fall_pitch_threshold_deg);
    GET_JSON_NUMBER_INT(obj, "fall_threshold_duration_ms", b.fall_threshold_duration_ms);
    GET_JSON_NUMBER_DOUBLE(obj, "recovery_pitch_threshold_deg", b.recovery_pitch_threshold_deg);
    GET_JSON_NUMBER_INT(obj, "recovery_hold_duration_ms", b.recovery_hold_duration_ms);
    GET_JSON_NUMBER_INT(obj, "battery_oversampling_count", b.battery_oversampling_count);
    GET_JSON_NUMBER_INT(obj, "battery_read_interval_ms", b.battery_read_interval_ms);
    GET_JSON_NUMBER_INT(obj, "imu_health_i2c_fail_threshold", b.imu_health_i2c_fail_threshold);
    GET_JSON_NUMBER_INT(obj, "imu_health_no_data_threshold", b.imu_health_no_data_threshold);
    GET_JSON_NUMBER_INT(obj, "imu_health_data_timeout_ms", b.imu_health_data_timeout_ms);
    return true; // Basic deserialization done, validation happens elsewhere
}

cJSON* JsonConfigParser::serializeDimensions(const RobotDimensionsConfig& d) const {
    cJSON *obj = cJSON_CreateObject(); if (!obj) return nullptr;
    cJSON_AddNumberToObject(obj, "wheelbase_m", d.wheelbase_m);
    return obj;
}

bool JsonConfigParser::deserializeDimensions(cJSON* obj, RobotDimensionsConfig& d) const {
    if (!obj) return false;
    GET_JSON_NUMBER_DOUBLE(obj, "wheelbase_m", d.wheelbase_m);
    return true;
}

cJSON* JsonConfigParser::serializeWeb(const WebServerConfig& w) const {
    cJSON *obj = cJSON_CreateObject(); if (!obj) return nullptr;
    cJSON_AddNumberToObject(obj, "telemetry_buffer_size", w.telemetry_buffer_size);
    cJSON_AddNumberToObject(obj, "max_config_post_size", w.max_config_post_size);
    return obj;
}

bool JsonConfigParser::deserializeWeb(cJSON* obj, WebServerConfig& w) const {
    if (!obj) return false;
    GET_JSON_NUMBER_INT(obj, "telemetry_buffer_size", w.telemetry_buffer_size);
    GET_JSON_NUMBER_INT(obj, "max_config_post_size", w.max_config_post_size);
    return true;
}

// --- Main Serialize/Deserialize Functions (Updated) ---

esp_err_t JsonConfigParser::serialize(const ConfigData& config, std::string& output) const {
    ESP_LOGD(TAG, "Serializing ConfigData to JSON");
    auto cjson_deleter = [](cJSON* ptr){ if(ptr) cJSON_Delete(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root_ptr(cJSON_CreateObject(), cjson_deleter);
    cJSON *root = root_ptr.get();
    if (!root) {
        ESP_LOGE(TAG, "Failed create root JSON object");
        return ESP_FAIL;
    }

    cJSON *section = nullptr; // Reusable pointer

    // Macro to simplify section creation and addition
    #define ADD_SECTION(section_name, create_func) \
        do { \
            section = create_func; \
            if (!section) { \
                ESP_LOGE(TAG, "Failed to create '%s' section object", section_name); \
                return ESP_FAIL; /* root_ptr cleans up root */ \
            } \
            if (!cJSON_AddItemToObject(root, section_name, section)) { \
                ESP_LOGE(TAG, "Failed to add '%s' section to root", section_name); \
                cJSON_Delete(section); /* Clean up the section manually */ \
                return ESP_FAIL; /* root_ptr cleans up root */ \
            } \
            section = nullptr; /* Ownership transferred, reset pointer */ \
        } while(0)

    // --- Config Version ---
    cJSON_AddNumberToObject(root, "config_version", config.config_version);

    // --- WiFi ---
    section = cJSON_CreateObject();
    if (!section) { ESP_LOGE(TAG, "Failed create wifi obj"); return ESP_FAIL; }
    cJSON_AddStringToObject(section, "ssid", config.wifi.ssid.c_str());
    cJSON_AddStringToObject(section, "password", config.wifi.password.c_str());
    if (!cJSON_AddItemToObject(root, "wifi", section)) { ESP_LOGE(TAG, "Failed add wifi obj"); cJSON_Delete(section); return ESP_FAIL; }
    section = nullptr;

    // --- Main Loop ---
    section = cJSON_CreateObject();
    if (!section) { ESP_LOGE(TAG, "Failed create mainLoop obj"); return ESP_FAIL; }
    cJSON_AddNumberToObject(section, "interval_ms", config.mainLoop.interval_ms);
    if (!cJSON_AddItemToObject(root, "mainLoop", section)) { ESP_LOGE(TAG, "Failed add mainLoop obj"); cJSON_Delete(section); return ESP_FAIL; }
    section = nullptr;

    // --- Control ---
    ADD_SECTION("control", serializeControl(config.control));

    // --- IMU ---
    section = cJSON_CreateObject(); if (!section) { ESP_LOGE(TAG, "Failed create imu obj"); return ESP_FAIL; }
    cJSON_AddNumberToObject(section, "i2c_port", config.imu.i2c_port);
    cJSON_AddNumberToObject(section, "sda_pin", config.imu.sda_pin);
    cJSON_AddNumberToObject(section, "scl_pin", config.imu.scl_pin);
    cJSON_AddNumberToObject(section, "device_address", config.imu.device_address);
    cJSON_AddNumberToObject(section, "i2c_freq_hz", config.imu.i2c_freq_hz);
    cJSON_AddNumberToObject(section, "int_pin", config.imu.int_pin);
    cJSON_AddBoolToObject(section, "interrupt_active_high", config.imu.interrupt_active_high);
    cJSON_AddNumberToObject(section, "accel_range", config.imu.accel_range);
    cJSON_AddNumberToObject(section, "gyro_range", config.imu.gyro_range);
    cJSON_AddNumberToObject(section, "dlpf_config", config.imu.dlpf_config);
    cJSON_AddNumberToObject(section, "sample_rate_divisor", config.imu.sample_rate_divisor);
    cJSON_AddNumberToObject(section, "calibration_samples", config.imu.calibration_samples);
    cJSON_AddNumberToObject(section, "comp_filter_alpha", config.imu.comp_filter_alpha);
    cJSON_AddNumberToObject(section, "fifo_read_threshold", config.imu.fifo_read_threshold);
    cJSON_AddNumberToObject(section, "gyro_offset_x", config.imu.gyro_offset_x); // Add offsets
    cJSON_AddNumberToObject(section, "gyro_offset_y", config.imu.gyro_offset_y);
    cJSON_AddNumberToObject(section, "gyro_offset_z", config.imu.gyro_offset_z);
    if (!cJSON_AddItemToObject(root, "imu", section)) { ESP_LOGE(TAG, "Failed add imu obj"); cJSON_Delete(section); return ESP_FAIL; }
    section = nullptr;

    // --- Encoder ---
    section = cJSON_CreateObject(); if (!section) { ESP_LOGE(TAG, "Failed create encoder obj"); return ESP_FAIL; }
    cJSON_AddNumberToObject(section, "left_pin_a", config.encoder.left_pin_a);
    cJSON_AddNumberToObject(section, "left_pin_b", config.encoder.left_pin_b);
    cJSON_AddNumberToObject(section, "right_pin_a", config.encoder.right_pin_a);
    cJSON_AddNumberToObject(section, "right_pin_b", config.encoder.right_pin_b);
    cJSON_AddNumberToObject(section, "pcnt_high_limit", config.encoder.pcnt_high_limit);
    cJSON_AddNumberToObject(section, "pcnt_low_limit", config.encoder.pcnt_low_limit);
    cJSON_AddNumberToObject(section, "pcnt_filter_ns", config.encoder.pcnt_filter_ns);
    cJSON_AddNumberToObject(section, "pulses_per_revolution_motor", config.encoder.pulses_per_revolution_motor);
    cJSON_AddNumberToObject(section, "gear_ratio", config.encoder.gear_ratio);
    cJSON_AddNumberToObject(section, "wheel_diameter_mm", config.encoder.wheel_diameter_mm);
    cJSON_AddNumberToObject(section, "speed_filter_alpha", config.encoder.speed_filter_alpha);
    if (!cJSON_AddItemToObject(root, "encoder", section)) { ESP_LOGE(TAG, "Failed add encoder obj"); cJSON_Delete(section); return ESP_FAIL; }
    section = nullptr;

    // --- Motor ---
    section = cJSON_CreateObject(); if (!section) { ESP_LOGE(TAG, "Failed create motor obj"); return ESP_FAIL; }
    cJSON_AddNumberToObject(section, "left_pin_in1", config.motor.left_pin_in1);
    cJSON_AddNumberToObject(section, "left_pin_in2", config.motor.left_pin_in2);
    cJSON_AddNumberToObject(section, "left_channel_1", config.motor.left_channel_1);
    cJSON_AddNumberToObject(section, "left_channel_2", config.motor.left_channel_2);
    cJSON_AddNumberToObject(section, "right_pin_in1", config.motor.right_pin_in1);
    cJSON_AddNumberToObject(section, "right_pin_in2", config.motor.right_pin_in2);
    cJSON_AddNumberToObject(section, "right_channel_1", config.motor.right_channel_1);
    cJSON_AddNumberToObject(section, "right_channel_2", config.motor.right_channel_2);
    cJSON_AddNumberToObject(section, "timer_num", config.motor.timer_num);
    cJSON_AddNumberToObject(section, "speed_mode", config.motor.speed_mode);
    cJSON_AddNumberToObject(section, "pwm_frequency_hz", config.motor.pwm_frequency_hz);
    cJSON_AddNumberToObject(section, "duty_resolution", config.motor.duty_resolution);
    cJSON_AddNumberToObject(section, "deadzone_duty", config.motor.deadzone_duty);
    if (!cJSON_AddItemToObject(root, "motor", section)) { ESP_LOGE(TAG, "Failed add motor obj"); cJSON_Delete(section); return ESP_FAIL; }
    section = nullptr;

    // --- Battery ---
    section = cJSON_CreateObject(); if (!section) { ESP_LOGE(TAG, "Failed create battery obj"); return ESP_FAIL; }
    cJSON_AddNumberToObject(section, "adc_pin", config.battery.adc_pin);
    cJSON_AddNumberToObject(section, "voltage_divider_ratio", config.battery.voltage_divider_ratio);
    cJSON_AddNumberToObject(section, "voltage_max", config.battery.voltage_max);
    cJSON_AddNumberToObject(section, "voltage_min", config.battery.voltage_min);
    cJSON_AddNumberToObject(section, "adc_bitwidth", config.battery.adc_bitwidth);
    cJSON_AddNumberToObject(section, "adc_atten", config.battery.adc_atten);
    if (!cJSON_AddItemToObject(root, "battery", section)) { ESP_LOGE(TAG, "Failed add battery obj"); cJSON_Delete(section); return ESP_FAIL; }
    section = nullptr;

    // --- System Behavior ---
    ADD_SECTION("behavior", serializeBehavior(config.behavior));
    // --- Robot Dimensions ---
    ADD_SECTION("dimensions", serializeDimensions(config.dimensions));
    // --- Web Server ---
    ADD_SECTION("web", serializeWeb(config.web));

    // --- PIDs ---
    ADD_SECTION("pid_angle", serializePid(config.pid_angle));
    ADD_SECTION("pid_speed_left", serializePid(config.pid_speed_left));
    ADD_SECTION("pid_speed_right", serializePid(config.pid_speed_right));
    ADD_SECTION("pid_yaw_rate", serializePid(config.pid_yaw_rate));

    // Clean up the macro definition
    #undef ADD_SECTION

    // --- Finalize JSON string ---
    { // Scope for json_str_ptr
        auto cjson_print_deleter = [](char* ptr){ if(ptr) free(ptr); };
        std::unique_ptr<char, decltype(cjson_print_deleter)> json_str_ptr(cJSON_Print(root), cjson_print_deleter);
        char *json_str = json_str_ptr.get();
        if (!json_str) { ESP_LOGE(TAG, "Failed print JSON"); return ESP_FAIL; }
        output = std::string(json_str);
    } // json_str_ptr goes out of scope and frees memory

    ESP_LOGD(TAG, "Serialization successful");
    return ESP_OK;
}


esp_err_t JsonConfigParser::deserialize(const std::string& input, ConfigData& configOutput) const {
    ESP_LOGD(TAG, "Deserializing JSON to ConfigData");
    auto cjson_deleter = [](cJSON* ptr){ if(ptr) cJSON_Delete(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root_ptr(cJSON_Parse(input.c_str()), cjson_deleter);
    cJSON* root = root_ptr.get();

    if (!root) {
        const char *error_ptr = cJSON_GetErrorPtr();
        ESP_LOGE(TAG, "JSON Parse Error: %s", error_ptr ? error_ptr : "Unknown");
        return ESP_FAIL;
    }

    ConfigData tempConfig; // Start with defaults
    bool pid_success = true;
    bool control_success = true;
    bool behavior_success = true;
    bool dimensions_success = true;
    bool web_success = true;

    // --- Config Version ---
    cJSON *version_item = cJSON_GetObjectItem(root, "config_version");
    if (version_item && cJSON_IsNumber(version_item)) {
        tempConfig.config_version = version_item->valueint;
    } else {
        ESP_LOGW(TAG, "'config_version' missing or invalid. Using default %d.", tempConfig.config_version);
    }

    // --- WiFi ---
    cJSON *wifi_section = cJSON_GetObjectItem(root, "wifi");
    if (wifi_section) {
        GET_JSON_STRING(wifi_section, "ssid", tempConfig.wifi.ssid);
        GET_JSON_STRING(wifi_section, "password", tempConfig.wifi.password);
    } else { ESP_LOGW(TAG, "'wifi' section missing."); }

    // --- Main Loop ---
    cJSON *main_loop_section = cJSON_GetObjectItem(root, "mainLoop");
    if (main_loop_section) {
        GET_JSON_NUMBER_INT(main_loop_section, "interval_ms", tempConfig.mainLoop.interval_ms);
    } else { ESP_LOGW(TAG, "'mainLoop' section missing."); }

    // --- Control ---
    cJSON *control_section = cJSON_GetObjectItem(root, "control");
    if (control_section) {
        if (!deserializeControl(control_section, tempConfig.control)) control_success = false;
    } else { ESP_LOGW(TAG, "'control' section missing."); control_success = false; }

    // --- IMU ---
    cJSON *imu_section = cJSON_GetObjectItem(root, "imu");
    if (imu_section) {
        GET_JSON_NUMBER_INT_CAST(imu_section, "i2c_port", tempConfig.imu.i2c_port, i2c_port_t);
        GET_JSON_NUMBER_INT_CAST(imu_section, "sda_pin", tempConfig.imu.sda_pin, gpio_num_t);
        GET_JSON_NUMBER_INT_CAST(imu_section, "scl_pin", tempConfig.imu.scl_pin, gpio_num_t);
        GET_JSON_NUMBER_INT(imu_section, "device_address", tempConfig.imu.device_address);
        GET_JSON_NUMBER_INT(imu_section, "i2c_freq_hz", tempConfig.imu.i2c_freq_hz);
        GET_JSON_NUMBER_INT_CAST(imu_section, "int_pin", tempConfig.imu.int_pin, gpio_num_t);
        GET_JSON_BOOL(imu_section, "interrupt_active_high", tempConfig.imu.interrupt_active_high);
        GET_JSON_NUMBER_INT(imu_section, "accel_range", tempConfig.imu.accel_range);
        GET_JSON_NUMBER_INT(imu_section, "gyro_range", tempConfig.imu.gyro_range);
        GET_JSON_NUMBER_INT(imu_section, "dlpf_config", tempConfig.imu.dlpf_config);
        GET_JSON_NUMBER_INT(imu_section, "sample_rate_divisor", tempConfig.imu.sample_rate_divisor);
        GET_JSON_NUMBER_INT(imu_section, "calibration_samples", tempConfig.imu.calibration_samples);
        GET_JSON_NUMBER_DOUBLE(imu_section, "comp_filter_alpha", tempConfig.imu.comp_filter_alpha);
        GET_JSON_NUMBER_INT(imu_section, "fifo_read_threshold", tempConfig.imu.fifo_read_threshold);
        GET_JSON_NUMBER_DOUBLE(imu_section, "gyro_offset_x", tempConfig.imu.gyro_offset_x);
        GET_JSON_NUMBER_DOUBLE(imu_section, "gyro_offset_y", tempConfig.imu.gyro_offset_y);
        GET_JSON_NUMBER_DOUBLE(imu_section, "gyro_offset_z", tempConfig.imu.gyro_offset_z);
    } else { ESP_LOGW(TAG, "'imu' section missing."); }

    // --- Encoder ---
    cJSON *encoder_section = cJSON_GetObjectItem(root, "encoder");
    if (encoder_section) {
        GET_JSON_NUMBER_INT_CAST(encoder_section, "left_pin_a", tempConfig.encoder.left_pin_a, gpio_num_t);
        GET_JSON_NUMBER_INT_CAST(encoder_section, "left_pin_b", tempConfig.encoder.left_pin_b, gpio_num_t);
        GET_JSON_NUMBER_INT_CAST(encoder_section, "right_pin_a", tempConfig.encoder.right_pin_a, gpio_num_t);
        GET_JSON_NUMBER_INT_CAST(encoder_section, "right_pin_b", tempConfig.encoder.right_pin_b, gpio_num_t);
        GET_JSON_NUMBER_INT(encoder_section, "pcnt_high_limit", tempConfig.encoder.pcnt_high_limit);
        GET_JSON_NUMBER_INT(encoder_section, "pcnt_low_limit", tempConfig.encoder.pcnt_low_limit);
        GET_JSON_NUMBER_INT(encoder_section, "pcnt_filter_ns", tempConfig.encoder.pcnt_filter_ns);
        GET_JSON_NUMBER_DOUBLE(encoder_section, "pulses_per_revolution_motor", tempConfig.encoder.pulses_per_revolution_motor);
        GET_JSON_NUMBER_DOUBLE(encoder_section, "gear_ratio", tempConfig.encoder.gear_ratio);
        GET_JSON_NUMBER_DOUBLE(encoder_section, "wheel_diameter_mm", tempConfig.encoder.wheel_diameter_mm);
        GET_JSON_NUMBER_DOUBLE(encoder_section, "speed_filter_alpha", tempConfig.encoder.speed_filter_alpha);
    } else { ESP_LOGW(TAG, "'encoder' section missing."); }

    // --- Motor ---
    cJSON *motor_section = cJSON_GetObjectItem(root, "motor");
    if (motor_section) {
        GET_JSON_NUMBER_INT_CAST(motor_section, "left_pin_in1", tempConfig.motor.left_pin_in1, gpio_num_t);
        GET_JSON_NUMBER_INT_CAST(motor_section, "left_pin_in2", tempConfig.motor.left_pin_in2, gpio_num_t);
        GET_JSON_NUMBER_INT_CAST(motor_section, "left_channel_1", tempConfig.motor.left_channel_1, ledc_channel_t);
        GET_JSON_NUMBER_INT_CAST(motor_section, "left_channel_2", tempConfig.motor.left_channel_2, ledc_channel_t);
        GET_JSON_NUMBER_INT_CAST(motor_section, "right_pin_in1", tempConfig.motor.right_pin_in1, gpio_num_t);
        GET_JSON_NUMBER_INT_CAST(motor_section, "right_pin_in2", tempConfig.motor.right_pin_in2, gpio_num_t);
        GET_JSON_NUMBER_INT_CAST(motor_section, "right_channel_1", tempConfig.motor.right_channel_1, ledc_channel_t);
        GET_JSON_NUMBER_INT_CAST(motor_section, "right_channel_2", tempConfig.motor.right_channel_2, ledc_channel_t);
        GET_JSON_NUMBER_INT_CAST(motor_section, "timer_num", tempConfig.motor.timer_num, ledc_timer_t);
        GET_JSON_NUMBER_INT_CAST(motor_section, "speed_mode", tempConfig.motor.speed_mode, ledc_mode_t);
        GET_JSON_NUMBER_INT(motor_section, "pwm_frequency_hz", tempConfig.motor.pwm_frequency_hz);
        GET_JSON_NUMBER_INT_CAST(motor_section, "duty_resolution", tempConfig.motor.duty_resolution, ledc_timer_bit_t);
        GET_JSON_NUMBER_INT(motor_section, "deadzone_duty", tempConfig.motor.deadzone_duty);
    } else { ESP_LOGW(TAG, "'motor' section missing."); }

     // --- Battery ---
    cJSON *battery_section = cJSON_GetObjectItem(root, "battery");
    if (battery_section) {
        GET_JSON_NUMBER_INT_CAST(battery_section, "adc_pin", tempConfig.battery.adc_pin, gpio_num_t);
        GET_JSON_NUMBER_DOUBLE(battery_section, "voltage_divider_ratio", tempConfig.battery.voltage_divider_ratio);
        GET_JSON_NUMBER_DOUBLE(battery_section, "voltage_max", tempConfig.battery.voltage_max);
        GET_JSON_NUMBER_DOUBLE(battery_section, "voltage_min", tempConfig.battery.voltage_min);
        GET_JSON_NUMBER_INT_CAST(battery_section, "adc_bitwidth", tempConfig.battery.adc_bitwidth, adc_bitwidth_t);
        GET_JSON_NUMBER_INT_CAST(battery_section, "adc_atten", tempConfig.battery.adc_atten, adc_atten_t);
    } else { ESP_LOGW(TAG, "'battery' section missing."); }

    // --- System Behavior ---
    cJSON *behavior_section = cJSON_GetObjectItem(root, "behavior");
    if (behavior_section) {
        if (!deserializeBehavior(behavior_section, tempConfig.behavior)) behavior_success = false;
    } else { ESP_LOGW(TAG, "'behavior' section missing."); behavior_success = false; }
    // --- Robot Dimensions ---
    cJSON *dimensions_section = cJSON_GetObjectItem(root, "dimensions");
    if (dimensions_section) {
        if (!deserializeDimensions(dimensions_section, tempConfig.dimensions)) dimensions_success = false;
    } else { ESP_LOGW(TAG, "'dimensions' section missing."); dimensions_success = false; }
    // --- Web Server ---
    cJSON *web_section = cJSON_GetObjectItem(root, "web");
    if (web_section) {
        if (!deserializeWeb(web_section, tempConfig.web)) web_success = false;
    } else { ESP_LOGW(TAG, "'web' section missing."); web_success = false; }

    // --- PIDs ---
    cJSON *pid_section;
    pid_section = cJSON_GetObjectItem(root, "pid_angle");
    if (pid_section && !deserializePid(pid_section, tempConfig.pid_angle)) pid_success = false; else if (!pid_section) { ESP_LOGW(TAG, "'pid_angle' missing."); pid_success = false; }

    pid_section = cJSON_GetObjectItem(root, "pid_speed_left");
    if (pid_section && !deserializePid(pid_section, tempConfig.pid_speed_left)) pid_success = false; else if (!pid_section) { ESP_LOGW(TAG, "'pid_speed_left' missing."); pid_success = false; }

    pid_section = cJSON_GetObjectItem(root, "pid_speed_right");
    if (pid_section && !deserializePid(pid_section, tempConfig.pid_speed_right)) pid_success = false; else if (!pid_section) { ESP_LOGW(TAG, "'pid_speed_right' missing."); pid_success = false; }

    pid_section = cJSON_GetObjectItem(root, "pid_yaw_rate");
    if (pid_section && !deserializePid(pid_section, tempConfig.pid_yaw_rate)) pid_success = false; else if (!pid_section) { ESP_LOGW(TAG, "'pid_yaw_rate' missing."); pid_success = false; }

    // Cleanup macros
    #undef GET_JSON_STRING
    #undef GET_JSON_NUMBER_INT
    #undef GET_JSON_NUMBER_INT_CAST
    #undef GET_JSON_NUMBER_DOUBLE
    #undef GET_JSON_BOOL

    // Only update the output reference if all critical sections were parsed ok
    if (pid_success && control_success && behavior_success && dimensions_success && web_success) {
        configOutput = tempConfig;
        ESP_LOGD(TAG, "Deserialization successful.");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Deserialization failed due to errors parsing critical/required sections.");
        return ESP_FAIL;
    }
}

// Helper to serialize a PIDConfig struct
cJSON* JsonConfigParser::serializePid(const PIDConfig& pidConfig) const {
    cJSON *pid_obj = cJSON_CreateObject();
    if (!pid_obj) return nullptr;
    cJSON_AddNumberToObject(pid_obj, "kp", pidConfig.pid_kp);
    cJSON_AddNumberToObject(pid_obj, "ki", pidConfig.pid_ki);
    cJSON_AddNumberToObject(pid_obj, "kd", pidConfig.pid_kd);
    cJSON_AddNumberToObject(pid_obj, "output_min", pidConfig.pid_output_min);
    cJSON_AddNumberToObject(pid_obj, "output_max", pidConfig.pid_output_max);
    cJSON_AddNumberToObject(pid_obj, "iterm_min", pidConfig.pid_iterm_min);
    cJSON_AddNumberToObject(pid_obj, "iterm_max", pidConfig.pid_iterm_max);
    return pid_obj;
}

// Helper to deserialize into a PIDConfig struct
bool JsonConfigParser::deserializePid(cJSON* pid_obj, PIDConfig& pidConfigOutput) const {
    bool ok = true;
    cJSON *item;
    #define PARSE_PID_FIELD(key, target) \
        item = cJSON_GetObjectItem(pid_obj, key); \
        if (item && cJSON_IsNumber(item)) target = item->valuedouble; else ok = false
    PARSE_PID_FIELD("kp", pidConfigOutput.pid_kp);
    PARSE_PID_FIELD("ki", pidConfigOutput.pid_ki);
    PARSE_PID_FIELD("kd", pidConfigOutput.pid_kd);
    PARSE_PID_FIELD("output_min", pidConfigOutput.pid_output_min);
    PARSE_PID_FIELD("output_max", pidConfigOutput.pid_output_max);
    PARSE_PID_FIELD("iterm_min", pidConfigOutput.pid_iterm_min);
    PARSE_PID_FIELD("iterm_max", pidConfigOutput.pid_iterm_max);
    #undef PARSE_PID_FIELD
    if (!ok) { ESP_LOGW(TAG, "Missing or invalid field in PID config section."); }
    return ok;
}

// CONTROL CONFIG HELPERS
cJSON* JsonConfigParser::serializeControl(const ControlConfig& controlConfig) const {
    cJSON *control_obj = cJSON_CreateObject();
    if (!control_obj) return nullptr;
    cJSON_AddNumberToObject(control_obj, "joystick_exponent", controlConfig.joystick_exponent);
    cJSON_AddNumberToObject(control_obj, "max_target_pitch_offset_deg", controlConfig.max_target_pitch_offset_deg);
    return control_obj;
}

bool JsonConfigParser::deserializeControl(cJSON* control_obj, ControlConfig& controlConfigOutput) const {
    bool ok = true;
    cJSON *item;
    item = cJSON_GetObjectItem(control_obj, "joystick_exponent");
    if (item && cJSON_IsNumber(item)) {
        controlConfigOutput.joystick_exponent = item->valuedouble;
    } else { ok = false; ESP_LOGW(TAG, "Missing/invalid 'joystick_exponent'"); }
    item = cJSON_GetObjectItem(control_obj, "max_target_pitch_offset_deg");
    if (item && cJSON_IsNumber(item)) {
        controlConfigOutput.max_target_pitch_offset_deg = item->valuedouble;
    } else { ok = false; ESP_LOGW(TAG, "Missing/invalid 'max_target_pitch_offset_deg'"); }
    if (!ok) { ESP_LOGW(TAG, "Error(s) parsing Control config."); }
    return ok;
}
