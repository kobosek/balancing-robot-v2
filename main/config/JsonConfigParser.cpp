#include "JsonConfigParser.hpp"         // Relative path within module's include dir
#include "cJSON.h"
#include "esp_log.h"
#include <string>
#include <vector>                       // Include vector for alternative approach if needed
#include <memory>                       // For unique_ptr
#include "ConfigData.hpp"

esp_err_t JsonConfigParser::serialize(const ConfigData& config, std::string& output) const {
    ESP_LOGD(TAG, "Serializing ConfigData to JSON");

    // Use unique_ptr with a custom deleter for automatic cleanup of root
    auto cjson_deleter = [](cJSON* ptr){ if(ptr) cJSON_Delete(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root_ptr(cJSON_CreateObject(), cjson_deleter);
    cJSON *root = root_ptr.get(); // Get raw pointer for cJSON API

    if (!root) {
        ESP_LOGE(TAG, "Failed to create root cJSON object");
        return ESP_FAIL;
    }

    cJSON *section = nullptr; // Reusable pointer for sections

    // --- Serialize WiFi ---
    section = cJSON_CreateObject();
    if (!section) { ESP_LOGE(TAG, "Failed create wifi obj"); return ESP_FAIL; } // Early exit cleans up root via unique_ptr
    cJSON_AddStringToObject(section, "ssid", config.wifi.ssid.c_str());
    cJSON_AddStringToObject(section, "password", config.wifi.password.c_str());
    if (!cJSON_AddItemToObject(root, "wifi", section)) { // Check AddItem success
         ESP_LOGE(TAG, "Failed add wifi obj to root"); cJSON_Delete(section); return ESP_FAIL;
    }
    section = nullptr; // Reset pointer as 'section' ownership transferred to 'root'

    // --- Serialize Main Loop ---
    section = cJSON_CreateObject();
    if (!section) { ESP_LOGE(TAG, "Failed create mainLoop obj"); return ESP_FAIL; }
    cJSON_AddNumberToObject(section, "interval_ms", config.mainLoop.interval_ms);
     if (!cJSON_AddItemToObject(root, "mainLoop", section)) {
         ESP_LOGE(TAG, "Failed add mainLoop obj to root"); cJSON_Delete(section); return ESP_FAIL;
    }
    section = nullptr;

    // --- Serialize IMU ---
    section = cJSON_CreateObject();
    if (!section) { ESP_LOGE(TAG, "Failed create imu obj"); return ESP_FAIL; }
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
     if (!cJSON_AddItemToObject(root, "imu", section)) {
         ESP_LOGE(TAG, "Failed add imu obj to root"); cJSON_Delete(section); return ESP_FAIL;
    }
    section = nullptr;

    // --- Serialize Encoder ---
    section = cJSON_CreateObject();
    if (!section) { ESP_LOGE(TAG, "Failed create encoder obj"); return ESP_FAIL; }
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
    if (!cJSON_AddItemToObject(root, "encoder", section)) {
         ESP_LOGE(TAG, "Failed add encoder obj to root"); cJSON_Delete(section); return ESP_FAIL;
    }
    section = nullptr;

    // --- Serialize Motor ---
    section = cJSON_CreateObject();
    if (!section) { ESP_LOGE(TAG, "Failed create motor obj"); return ESP_FAIL; }
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
     if (!cJSON_AddItemToObject(root, "motor", section)) {
         ESP_LOGE(TAG, "Failed add motor obj to root"); cJSON_Delete(section); return ESP_FAIL;
    }
    section = nullptr;

    // --- Serialize Battery ---
    section = cJSON_CreateObject();
    if (!section) { ESP_LOGE(TAG, "Failed create battery obj"); return ESP_FAIL; }
    cJSON_AddNumberToObject(section, "adc_pin", config.battery.adc_pin);
    cJSON_AddNumberToObject(section, "voltage_divider_ratio", config.battery.voltage_divider_ratio);
    cJSON_AddNumberToObject(section, "voltage_max", config.battery.voltage_max);
    cJSON_AddNumberToObject(section, "voltage_min", config.battery.voltage_min);
    if (!cJSON_AddItemToObject(root, "battery", section)) {
         ESP_LOGE(TAG, "Failed add battery obj to root"); cJSON_Delete(section); return ESP_FAIL;
    }
    section = nullptr;

    // --- Serialize PID Angle ---
    section = serializePid(config.anglePid); // Use helper
    if (!section) { ESP_LOGE(TAG, "Failed serialize pid_angle"); return ESP_FAIL; }
    if (!cJSON_AddItemToObject(root, "pid_angle", section)) {
         ESP_LOGE(TAG, "Failed add pid_angle obj to root"); cJSON_Delete(section); return ESP_FAIL;
    }
    section = nullptr;

    // --- Serialize PID Speed Left ---
    section = serializePid(config.speedPidLeft);
    if (!section) { ESP_LOGE(TAG, "Failed serialize pid_speed_left"); return ESP_FAIL; }
    if (!cJSON_AddItemToObject(root, "pid_speed_left", section)) {
         ESP_LOGE(TAG, "Failed add pid_speed_left obj to root"); cJSON_Delete(section); return ESP_FAIL;
    }
    section = nullptr;

    // --- Serialize PID Speed Right ---
    section = serializePid(config.speedPidRight);
    if (!section) { ESP_LOGE(TAG, "Failed serialize pid_speed_right"); return ESP_FAIL; }
    if (!cJSON_AddItemToObject(root, "pid_speed_right", section)) {
         ESP_LOGE(TAG, "Failed add pid_speed_right obj to root"); cJSON_Delete(section); return ESP_FAIL;
    }
    section = nullptr;

    // --- Finalize JSON string ---
    // Use unique_ptr for freeing the cJSON_Print output
    auto cjson_print_deleter = [](char* ptr){ if(ptr) free(ptr); };
    std::unique_ptr<char, decltype(cjson_print_deleter)> json_str_ptr(cJSON_Print(root), cjson_print_deleter);
    char *json_str = json_str_ptr.get();

    if (!json_str) {
        ESP_LOGE(TAG, "Failed to print cJSON to string");
        // root cleanup handled by root_ptr unique_ptr going out of scope
        return ESP_FAIL;
    }

    output = std::string(json_str);
    ESP_LOGD(TAG, "Serialization successful");
    // root cleanup handled by root_ptr unique_ptr going out of scope
    // json_str cleanup handled by json_str_ptr unique_ptr going out of scope
    return ESP_OK;
}


// --- Deserialization (remains mostly the same as previous correct version) ---
esp_err_t JsonConfigParser::deserialize(const std::string& input, ConfigData& configOutput) const {
    ESP_LOGD(TAG, "Deserializing JSON to ConfigData");

    // Use unique_ptr for automatic cleanup of root
    auto cjson_deleter = [](cJSON* ptr){ if(ptr) cJSON_Delete(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root_ptr(cJSON_Parse(input.c_str()), cjson_deleter);
    cJSON* root = root_ptr.get();

    if (root == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            ESP_LOGE(TAG, "JSON Parse Error before: %s", error_ptr);
        } else {
            ESP_LOGE(TAG, "JSON Parse Error: Unknown error");
        }
        return ESP_FAIL;
    }

    ConfigData tempConfig; // Start with defaults
    bool pid_success = true;

    // --- Deserialize WiFi ---
    cJSON *wifi_section = cJSON_GetObjectItem(root, "wifi");
    if (wifi_section) {
        cJSON *item = cJSON_GetObjectItem(wifi_section, "ssid");
        if (item && cJSON_IsString(item)) tempConfig.wifi.ssid = item->valuestring;
        item = cJSON_GetObjectItem(wifi_section, "password");
        if (item && cJSON_IsString(item)) tempConfig.wifi.password = item->valuestring;
    } else { ESP_LOGW(TAG, "'wifi' section not found in JSON, using defaults."); }

    // --- Deserialize Main Loop ---
    cJSON *main_loop_section = cJSON_GetObjectItem(root, "mainLoop");
    if (main_loop_section) {
        cJSON *item = cJSON_GetObjectItem(main_loop_section, "interval_ms");
        if (item && cJSON_IsNumber(item)) tempConfig.mainLoop.interval_ms = item->valueint;
    } else { ESP_LOGW(TAG, "'mainLoop' section not found in JSON, using defaults."); }

    // --- Deserialize IMU ---
    cJSON *imu_section = cJSON_GetObjectItem(root, "imu");
    if (imu_section) {
        cJSON *item; // Reuse item pointer
        item = cJSON_GetObjectItem(imu_section, "i2c_port"); if (item && cJSON_IsNumber(item)) tempConfig.imu.i2c_port = (i2c_port_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.i2c_port"); }
        item = cJSON_GetObjectItem(imu_section, "sda_pin"); if (item && cJSON_IsNumber(item)) tempConfig.imu.sda_pin = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.sda_pin"); }
        item = cJSON_GetObjectItem(imu_section, "scl_pin"); if (item && cJSON_IsNumber(item)) tempConfig.imu.scl_pin = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.scl_pin"); }
        item = cJSON_GetObjectItem(imu_section, "device_address"); if (item && cJSON_IsNumber(item)) tempConfig.imu.device_address = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.device_address"); }
        item = cJSON_GetObjectItem(imu_section, "i2c_freq_hz"); if (item && cJSON_IsNumber(item)) tempConfig.imu.i2c_freq_hz = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.i2c_freq_hz"); }
        item = cJSON_GetObjectItem(imu_section, "int_pin"); if (item && cJSON_IsNumber(item)) tempConfig.imu.int_pin = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.int_pin"); }
        item = cJSON_GetObjectItem(imu_section, "interrupt_active_high"); if (item && cJSON_IsBool(item)) tempConfig.imu.interrupt_active_high = cJSON_IsTrue(item); else { ESP_LOGW(TAG, "Missing/invalid imu.interrupt_active_high"); }
        item = cJSON_GetObjectItem(imu_section, "accel_range"); if (item && cJSON_IsNumber(item)) tempConfig.imu.accel_range = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.accel_range"); }
        item = cJSON_GetObjectItem(imu_section, "gyro_range"); if (item && cJSON_IsNumber(item)) tempConfig.imu.gyro_range = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.gyro_range"); }
        item = cJSON_GetObjectItem(imu_section, "dlpf_config"); if (item && cJSON_IsNumber(item)) tempConfig.imu.dlpf_config = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.dlpf_config"); }
        item = cJSON_GetObjectItem(imu_section, "sample_rate_divisor"); if (item && cJSON_IsNumber(item)) tempConfig.imu.sample_rate_divisor = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.sample_rate_divisor"); }
        item = cJSON_GetObjectItem(imu_section, "calibration_samples"); if (item && cJSON_IsNumber(item)) tempConfig.imu.calibration_samples = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.calibration_samples"); }
        item = cJSON_GetObjectItem(imu_section, "comp_filter_alpha"); if (item && cJSON_IsNumber(item)) tempConfig.imu.comp_filter_alpha = item->valuedouble; else { ESP_LOGW(TAG, "Missing/invalid imu.comp_filter_alpha"); }
        item = cJSON_GetObjectItem(imu_section, "fifo_read_threshold"); if (item && cJSON_IsNumber(item)) tempConfig.imu.fifo_read_threshold = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid imu.fifo_read_threshold"); }
    } else { ESP_LOGW(TAG, "'imu' section not found in JSON, using defaults."); }

    // --- Deserialize Encoder ---
    cJSON *encoder_section = cJSON_GetObjectItem(root, "encoder");
    if (encoder_section) {
        cJSON *item; // Reuse item pointer
        item = cJSON_GetObjectItem(encoder_section, "left_pin_a"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.left_pin_a = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid encoder.left_pin_a"); }
        item = cJSON_GetObjectItem(encoder_section, "left_pin_b"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.left_pin_b = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid encoder.left_pin_b"); }
        item = cJSON_GetObjectItem(encoder_section, "right_pin_a"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.right_pin_a = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid encoder.right_pin_a"); }
        item = cJSON_GetObjectItem(encoder_section, "right_pin_b"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.right_pin_b = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid encoder.right_pin_b"); }
        item = cJSON_GetObjectItem(encoder_section, "pcnt_high_limit"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.pcnt_high_limit = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid encoder.pcnt_high_limit"); }
        item = cJSON_GetObjectItem(encoder_section, "pcnt_low_limit"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.pcnt_low_limit = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid encoder.pcnt_low_limit"); }
        item = cJSON_GetObjectItem(encoder_section, "pcnt_filter_ns"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.pcnt_filter_ns = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid encoder.pcnt_filter_ns"); }
        item = cJSON_GetObjectItem(encoder_section, "pulses_per_revolution_motor"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.pulses_per_revolution_motor = item->valuedouble; else { ESP_LOGW(TAG, "Missing/invalid encoder.pulses_per_revolution_motor"); }
        item = cJSON_GetObjectItem(encoder_section, "gear_ratio"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.gear_ratio = item->valuedouble; else { ESP_LOGW(TAG, "Missing/invalid encoder.gear_ratio"); }
        item = cJSON_GetObjectItem(encoder_section, "wheel_diameter_mm"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.wheel_diameter_mm = item->valuedouble; else { ESP_LOGW(TAG, "Missing/invalid encoder.wheel_diameter_mm"); }
        item = cJSON_GetObjectItem(encoder_section, "speed_filter_alpha"); if (item && cJSON_IsNumber(item)) tempConfig.encoder.speed_filter_alpha = item->valuedouble; else { ESP_LOGW(TAG, "Missing/invalid encoder.speed_filter_alpha"); }
    } else { ESP_LOGW(TAG, "'encoder' section not found in JSON, using defaults."); }

    // --- Deserialize Motor ---
    cJSON *motor_section = cJSON_GetObjectItem(root, "motor");
    if (motor_section) {
         cJSON *item; // Reuse item pointer
        item = cJSON_GetObjectItem(motor_section, "left_pin_in1"); if (item && cJSON_IsNumber(item)) tempConfig.motor.left_pin_in1 = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.left_pin_in1"); }
        item = cJSON_GetObjectItem(motor_section, "left_pin_in2"); if (item && cJSON_IsNumber(item)) tempConfig.motor.left_pin_in2 = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.left_pin_in2"); }
        item = cJSON_GetObjectItem(motor_section, "left_channel_1"); if (item && cJSON_IsNumber(item)) tempConfig.motor.left_channel_1 = (ledc_channel_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.left_channel_1"); }
        item = cJSON_GetObjectItem(motor_section, "left_channel_2"); if (item && cJSON_IsNumber(item)) tempConfig.motor.left_channel_2 = (ledc_channel_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.left_channel_2"); }
        item = cJSON_GetObjectItem(motor_section, "right_pin_in1"); if (item && cJSON_IsNumber(item)) tempConfig.motor.right_pin_in1 = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.right_pin_in1"); }
        item = cJSON_GetObjectItem(motor_section, "right_pin_in2"); if (item && cJSON_IsNumber(item)) tempConfig.motor.right_pin_in2 = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.right_pin_in2"); }
        item = cJSON_GetObjectItem(motor_section, "right_channel_1"); if (item && cJSON_IsNumber(item)) tempConfig.motor.right_channel_1 = (ledc_channel_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.right_channel_1"); }
        item = cJSON_GetObjectItem(motor_section, "right_channel_2"); if (item && cJSON_IsNumber(item)) tempConfig.motor.right_channel_2 = (ledc_channel_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.right_channel_2"); }
        item = cJSON_GetObjectItem(motor_section, "timer_num"); if (item && cJSON_IsNumber(item)) tempConfig.motor.timer_num = (ledc_timer_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.timer_num"); }
        item = cJSON_GetObjectItem(motor_section, "speed_mode"); if (item && cJSON_IsNumber(item)) tempConfig.motor.speed_mode = (ledc_mode_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.speed_mode"); }
        item = cJSON_GetObjectItem(motor_section, "pwm_frequency_hz"); if (item && cJSON_IsNumber(item)) tempConfig.motor.pwm_frequency_hz = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.pwm_frequency_hz"); }
        item = cJSON_GetObjectItem(motor_section, "duty_resolution"); if (item && cJSON_IsNumber(item)) tempConfig.motor.duty_resolution = (ledc_timer_bit_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.duty_resolution"); }
        item = cJSON_GetObjectItem(motor_section, "deadzone_duty"); if (item && cJSON_IsNumber(item)) tempConfig.motor.deadzone_duty = item->valueint; else { ESP_LOGW(TAG, "Missing/invalid motor.deadzone_duty"); }
    } else { ESP_LOGW(TAG, "'motor' section not found in JSON, using defaults."); }

     // --- Deserialize Battery ---
    cJSON *battery_section = cJSON_GetObjectItem(root, "battery");
    if (battery_section) {
        cJSON *item; // Reuse item pointer
        item = cJSON_GetObjectItem(battery_section, "adc_pin"); if (item && cJSON_IsNumber(item)) tempConfig.battery.adc_pin = (gpio_num_t)item->valueint; else { ESP_LOGW(TAG, "Missing/invalid battery.adc_pin"); }
        item = cJSON_GetObjectItem(battery_section, "voltage_divider_ratio"); if (item && cJSON_IsNumber(item)) tempConfig.battery.voltage_divider_ratio = item->valuedouble; else { ESP_LOGW(TAG, "Missing/invalid battery.voltage_divider_ratio"); }
        item = cJSON_GetObjectItem(battery_section, "voltage_max"); if (item && cJSON_IsNumber(item)) tempConfig.battery.voltage_max = item->valuedouble; else { ESP_LOGW(TAG, "Missing/invalid battery.voltage_max"); }
        item = cJSON_GetObjectItem(battery_section, "voltage_min"); if (item && cJSON_IsNumber(item)) tempConfig.battery.voltage_min = item->valuedouble; else { ESP_LOGW(TAG, "Missing/invalid battery.voltage_min"); }
    } else { ESP_LOGW(TAG, "'battery' section not found in JSON, using defaults."); }

    // --- Deserialize PID Angle ---
    cJSON *pid_angle_section = cJSON_GetObjectItem(root, "pid_angle");
    if (pid_angle_section) {
        if (!deserializePid(pid_angle_section, tempConfig.anglePid)) pid_success = false;
    } else { ESP_LOGW(TAG, "'pid_angle' section not found, using defaults."); }

    // --- Deserialize PID Speed Left ---
    cJSON *pid_speed_left_section = cJSON_GetObjectItem(root, "pid_speed_left");
    if (pid_speed_left_section) {
        if (!deserializePid(pid_speed_left_section, tempConfig.speedPidLeft)) pid_success = false;
    } else { ESP_LOGW(TAG, "'pid_speed_left' section not found, using defaults."); }

    // --- Deserialize PID Speed Right ---
    cJSON *pid_speed_right_section = cJSON_GetObjectItem(root, "pid_speed_right");
    if (pid_speed_right_section) {
        if (!deserializePid(pid_speed_right_section, tempConfig.speedPidRight)) pid_success = false;
    } else { ESP_LOGW(TAG, "'pid_speed_right' section not found, using defaults."); }


    // root cleanup handled by root_ptr unique_ptr going out of scope

    // Only update the output reference if all critical sections (like PIDs) were parsed ok
    if (pid_success) {
        configOutput = tempConfig; // Copy successfully parsed data
        ESP_LOGD(TAG, "Deserialization successful (minor warnings may exist).");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Deserialization failed due to errors parsing critical PID sections.");
        return ESP_FAIL; // Indicate failure if PIDs couldn't be parsed
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
// Returns true on success, false if any field is missing/invalid
bool JsonConfigParser::deserializePid(cJSON* pid_obj, PIDConfig& pidConfigOutput) const {
    bool ok = true;
    cJSON *item;

    item = cJSON_GetObjectItem(pid_obj, "kp");
    if (item && cJSON_IsNumber(item)) pidConfigOutput.pid_kp = item->valuedouble; else ok = false;

    item = cJSON_GetObjectItem(pid_obj, "ki");
    if (item && cJSON_IsNumber(item)) pidConfigOutput.pid_ki = item->valuedouble; else ok = false;

    item = cJSON_GetObjectItem(pid_obj, "kd");
    if (item && cJSON_IsNumber(item)) pidConfigOutput.pid_kd = item->valuedouble; else ok = false;

    item = cJSON_GetObjectItem(pid_obj, "output_min");
    if (item && cJSON_IsNumber(item)) pidConfigOutput.pid_output_min = item->valuedouble; else ok = false;

    item = cJSON_GetObjectItem(pid_obj, "output_max");
    if (item && cJSON_IsNumber(item)) pidConfigOutput.pid_output_max = item->valuedouble; else ok = false;

    item = cJSON_GetObjectItem(pid_obj, "iterm_min");
    if (item && cJSON_IsNumber(item)) pidConfigOutput.pid_iterm_min = item->valuedouble; else ok = false;

    item = cJSON_GetObjectItem(pid_obj, "iterm_max");
    if (item && cJSON_IsNumber(item)) pidConfigOutput.pid_iterm_max = item->valuedouble; else ok = false;

    if (!ok) { ESP_LOGW(TAG, "Missing or invalid field in PID configuration section."); }
    return ok;
}