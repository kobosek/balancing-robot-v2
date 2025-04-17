// ================================================
// File: main/config/ConfigurationService.cpp
// ================================================
#include "ConfigurationService.hpp"     // Relative path within module's include dir
#include "ConfigUpdatedEvent.hpp"       // Found via INCLUDE_DIRS
#include "SPIFFSStorageService.hpp"     // Relative path within module's include dir
#include "JsonConfigParser.hpp"         // Relative path within module's include dir
#include "EventBus.hpp"                 // Found via INCLUDE_DIRS
#include "ConfigData.hpp"               // Found via INCLUDE_DIRS (needed for constructor/defaults)
#include "BaseEvent.hpp"                // Found via INCLUDE_DIRS (needed for publish)
#include "GyroOffsetsUpdatedEvent.hpp" // For handling gyro offset updates
#include <string>
#include "esp_log.h"                    // Moved from header

ConfigurationService::ConfigurationService(IStorageService& storage, IConfigParser& parser, EventBus& bus, const std::string& configKey) :
    m_storageService(storage),
    m_configParser(parser),
    m_eventBus(bus),
    m_configKey(configKey)
{}

esp_err_t ConfigurationService::init() {
    std::lock_guard<std::mutex> lock(m_mutex);
    ESP_LOGI(TAG, "Initializing Configuration Service. Loading from key '%s'", m_configKey.c_str());
    std::string rawData;
    esp_err_t ret = m_storageService.loadData(m_configKey, rawData);
    bool useDefaults = false;
    std::string reasonForDefaults;

    if (ret == ESP_OK) {
        if (rawData.empty()) {
            reasonForDefaults = "Config file empty";
            useDefaults = true;
        } else {
            ret = m_configParser.deserialize(rawData, m_configData);
            if (ret == ESP_OK) {
                std::string validationError;
                if (!validateConfig(m_configData, validationError)) {
                    reasonForDefaults = "Validation failed: " + validationError;
                    useDefaults = true;
                } else {
                    ESP_LOGI(TAG, "Configuration loaded, parsed, and validated successfully (Version: %d).", m_configData.config_version);
                }
            } else {
                reasonForDefaults = "JSON parse failed";
                useDefaults = true;
            }
        }
    } else if (ret == ESP_ERR_NOT_FOUND) {
        reasonForDefaults = "Config file not found";
        useDefaults = true;
    } else {
        reasonForDefaults = std::string("Storage load failed: ") + esp_err_to_name(ret);
        useDefaults = true;
    }

    if (useDefaults) {
        ESP_LOGW(TAG, "%s. Using default values.", reasonForDefaults.c_str());
        m_configData = ConfigData(); // Re-assign defaults
        ESP_LOGW(TAG, "Attempting to save default configuration (Version: %d) to storage...", m_configData.config_version);
        esp_err_t save_ret = saveInternal(); // Attempt to save defaults
        if (save_ret != ESP_OK) {
           ESP_LOGE(TAG, "Failed to save default configuration to '%s'.", m_configKey.c_str());
           // This might be serious, but we continue with defaults in memory
        } else {
            ESP_LOGI(TAG, "Default configuration saved successfully.");
        }
    }

    // Publish initial config state regardless of load source
    ESP_LOGI(TAG, "Publishing initial configuration state.");
    ConfigUpdatedEvent initialEvent(m_configData);
    m_eventBus.publish(initialEvent);

    return ESP_OK; // Return OK even if defaults were used, as the service is operational
}

esp_err_t ConfigurationService::save() {
    std::lock_guard<std::mutex> lock(m_mutex);
    // Optionally add validation before saving? Depends if internal state could become invalid.
    // std::string validationError;
    // if (!validateConfig(m_configData, validationError)) {
    //     ESP_LOGE(TAG, "Pre-save validation failed: %s. Aborting save.", validationError.c_str());
    //     return ESP_FAIL;
    // }
    return saveInternal();
}

esp_err_t ConfigurationService::updateConfigFromJson(const std::string& json) {
    ConfigData tempConfig; // Create a temporary config to parse into
    esp_err_t ret = m_configParser.deserialize(json, tempConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deserialize JSON for update: %s", esp_err_to_name(ret));
        return ret;
    }

    std::string validationError;
    if (!validateConfig(tempConfig, validationError)) {
        ESP_LOGE(TAG, "Validation failed for config update: %s", validationError.c_str());
        return ESP_FAIL; // Return specific validation failure (maybe a different error code?)
    }

    // Lock only when updating the internal state and saving
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_configData = tempConfig; // Update internal state
        ret = saveInternal();      // Attempt to save
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save configuration after update.");
            // Even if save fails, the config in memory *is* updated.
        } else {
             ESP_LOGI(TAG, "Configuration updated and saved successfully (Version: %d).", m_configData.config_version);
        }
    } // Mutex released

    // Publish event regardless of save outcome (config in memory changed)
    ESP_LOGI(TAG, "Publishing configuration update event.");
    ConfigUpdatedEvent event(m_configData); // Pass the updated config data
    m_eventBus.publish(event);

    return ret; // Return the result of the save operation
}

void ConfigurationService::updateImuGyroOffsets(float x, float y, float z) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_configData.imu.gyro_offset_x != x ||
        m_configData.imu.gyro_offset_y != y ||
        m_configData.imu.gyro_offset_z != z)
    {
        ESP_LOGI(TAG, "Updating persistent gyro offsets: X=%.4f Y=%.4f Z=%.4f", x, y, z);
        m_configData.imu.gyro_offset_x = x;
        m_configData.imu.gyro_offset_y = y;
        m_configData.imu.gyro_offset_z = z;
        // We save, but also publish a full config update so listeners like IMUService get the update
        if (saveInternal() == ESP_OK) {
            ESP_LOGI(TAG, "Publishing configuration update after saving gyro offsets.");
            ConfigUpdatedEvent event(m_configData); // Pass the updated config data
            m_eventBus.publish(event);
        }
    } else {
        ESP_LOGD(TAG, "Gyro offsets received but are unchanged. No save needed.");
    }
}

void ConfigurationService::subscribeToEvents(EventBus& bus) {
    ESP_LOGI(TAG, "ConfigurationService subscribing to events...");

    // Subscribe to gyro offset updates from IMUCalibrationService
    bus.subscribe(EventType::GYRO_OFFSETS_UPDATED, [this](const BaseEvent& ev) {
        const auto& offsetEvent = static_cast<const GyroOffsetsUpdatedEvent&>(ev);
        this->updateImuGyroOffsets(offsetEvent.x_dps, offsetEvent.y_dps, offsetEvent.z_dps);
    });

    ESP_LOGI(TAG, "ConfigurationService subscriptions complete.");
}

// --- Config Validation ---
bool ConfigurationService::validateConfig(const ConfigData& config, std::string& error) const {
    // Config Version
    if (config.config_version < 1 || config.config_version > 1000) {
        error = "config_version out of range (1-1000)"; return false;
    }
    // WiFiConfig
    if (config.wifi.ssid.empty() || config.wifi.ssid.length() > 32) {
        error = "WiFi SSID must be 1-32 characters"; return false;
    }
    // Allow empty password for open networks? For now, require 8-63 for WPA2.
    if (!config.wifi.password.empty() && (config.wifi.password.length() < 8 || config.wifi.password.length() > 63)) {
        error = "WiFi password must be 8-63 characters (or empty for open networks)"; return false;
    }
    // MainLoopConfig
    if (config.mainLoop.interval_ms < 1 || config.mainLoop.interval_ms > 1000) {
        error = "mainLoop.interval_ms out of range (1-1000)"; return false;
    }
    // ControlConfig
    if (config.control.joystick_exponent < 0.5f || config.control.joystick_exponent > 5.0f) {
        error = "control.joystick_exponent out of range (0.5-5.0)"; return false;
    }
    if (config.control.max_target_pitch_offset_deg < 0.0f || config.control.max_target_pitch_offset_deg > 90.0f) {
        error = "control.max_target_pitch_offset_deg out of range (0-90)"; return false;
    }
    // MPU6050Config (IMU)
    if (config.imu.i2c_port != 0 && config.imu.i2c_port != 1) {
        error = "imu.i2c_port must be 0 or 1"; return false;
    }
    // Basic GPIO range check (ESP32 specific)
    auto is_valid_gpio = [](int pin){ return pin >= 0 && pin <= 39; };
    if (!is_valid_gpio(config.imu.sda_pin) || !is_valid_gpio(config.imu.scl_pin) || (config.imu.int_pin != -1 && !is_valid_gpio(config.imu.int_pin))) {
        error = "imu pins (SDA, SCL, INT) must be valid GPIOs (0-39 or -1 for INT)"; return false;
    }
    if (config.imu.device_address < 0x08 || config.imu.device_address > 0x77) {
        error = "imu.device_address out of range (0x08-0x77)"; return false;
    }
    if (config.imu.i2c_freq_hz < 10000 || config.imu.i2c_freq_hz > 1000000) {
        error = "imu.i2c_freq_hz out of range (10k-1M)"; return false;
    }
    if (config.imu.accel_range < 0 || config.imu.accel_range > 3) {
        error = "imu.accel_range out of range (0-3)"; return false;
    }
    if (config.imu.gyro_range < 0 || config.imu.gyro_range > 3) {
        error = "imu.gyro_range out of range (0-3)"; return false;
    }
    if (config.imu.dlpf_config < 0 || config.imu.dlpf_config > 7) {
        error = "imu.dlpf_config out of range (0-7)"; return false;
    }
    if (config.imu.sample_rate_divisor < 0 || config.imu.sample_rate_divisor > 255) {
        error = "imu.sample_rate_divisor out of range (0-255)"; return false;
    }
    if (config.imu.calibration_samples < 10 || config.imu.calibration_samples > 10000) {
        error = "imu.calibration_samples out of range (10-10000)"; return false;
    }
    if (config.imu.comp_filter_alpha < 0.0f || config.imu.comp_filter_alpha > 1.0f) {
        error = "imu.comp_filter_alpha out of range (0.0-1.0)"; return false;
    }
    if (config.imu.fifo_read_threshold < 1 || config.imu.fifo_read_threshold > 1024) {
        error = "imu.fifo_read_threshold out of range (1-1024)"; return false;
    }
    // EncoderConfig
    if (!is_valid_gpio(config.encoder.left_pin_a) || !is_valid_gpio(config.encoder.left_pin_b) || !is_valid_gpio(config.encoder.right_pin_a) || !is_valid_gpio(config.encoder.right_pin_b)) {
        error = "encoder pins must be valid GPIOs (0-39)"; return false;
    }
    // PCNT limits can be negative
    if (config.encoder.pcnt_high_limit < -32768 || config.encoder.pcnt_high_limit > 32767 ||
        config.encoder.pcnt_low_limit < -32768 || config.encoder.pcnt_low_limit > 32767 ||
        config.encoder.pcnt_low_limit >= config.encoder.pcnt_high_limit) {
        error = "encoder.pcnt limits out of range [-32768, 32767] or low >= high"; return false;
    }
    if (config.encoder.pcnt_filter_ns < 0 || config.encoder.pcnt_filter_ns > 10000) {
        error = "encoder.pcnt_filter_ns out of range (0-10000)"; return false;
    }
    if (config.encoder.pulses_per_revolution_motor <= 0.0f || config.encoder.pulses_per_revolution_motor > 10000.0f) {
        error = "encoder.pulses_per_revolution_motor must be > 0 and reasonable (<10k)"; return false;
    }
    if (config.encoder.gear_ratio <= 0.0f || config.encoder.gear_ratio > 1000.0f) {
        error = "encoder.gear_ratio must be > 0 and reasonable (<1k)"; return false;
    }
    if (config.encoder.wheel_diameter_mm <= 0.0f || config.encoder.wheel_diameter_mm > 1000.0f) {
        error = "encoder.wheel_diameter_mm must be > 0 and reasonable (<1000)"; return false;
    }
    if (config.encoder.speed_filter_alpha < 0.0f || config.encoder.speed_filter_alpha > 1.0f) {
        error = "encoder.speed_filter_alpha out of range (0.0-1.0)"; return false;
    }
    // MotorConfig
    if (!is_valid_gpio(config.motor.left_pin_in1) || !is_valid_gpio(config.motor.left_pin_in2) || !is_valid_gpio(config.motor.right_pin_in1) || !is_valid_gpio(config.motor.right_pin_in2)) {
        error = "motor pins must be valid GPIOs (0-39)"; return false;
    }
    if (config.motor.pwm_frequency_hz < 1000 || config.motor.pwm_frequency_hz > 100000) {
        error = "motor.pwm_frequency_hz out of range (1k-100k)"; return false;
    }
    uint32_t max_duty_for_res = (1 << config.motor.duty_resolution) - 1;
    if (config.motor.deadzone_duty > max_duty_for_res) {
        error = "motor.deadzone_duty cannot exceed max duty for resolution"; return false;
    }
    // (Not validating LEDC enums like channel, timer, mode, resolution)
    // BatteryConfig
    if (config.battery.adc_pin != -1 && !is_valid_gpio(config.battery.adc_pin)) { // Allow -1 if not used? Check ADC driver capability
        error = "battery.adc_pin must be valid GPIO (0-39) or -1"; return false;
    }
    if (config.battery.voltage_divider_ratio < 0.1f || config.battery.voltage_divider_ratio > 100.0f) {
        error = "battery.voltage_divider_ratio out of range (0.1-100)"; return false;
    }
    if (config.battery.voltage_max < 0.0f || config.battery.voltage_max > 100.0f) {
        error = "battery.voltage_max out of range (0-100)"; return false;
    }
    if (config.battery.voltage_min < 0.0f || config.battery.voltage_min >= config.battery.voltage_max) {
        error = "battery.voltage_min out of range (0 - voltage_max)"; return false;
    }
    // PIDConfig (anglePid, speedPidLeft, speedPidRight, yawRatePid)
    // Using tighter, more realistic ranges now
    auto validatePID = [&](const PIDConfig& pid, const std::string& name) -> bool {
        if (pid.pid_kp < 0.0f || pid.pid_kp > 500.0f) { error = name + ".kp out of range [0, 500]"; return false; }
        if (pid.pid_ki < 0.0f || pid.pid_ki > 500.0f) { error = name + ".ki out of range [0, 500]"; return false; }
        if (pid.pid_kd < 0.0f || pid.pid_kd > 500.0f) { error = name + ".kd out of range [0, 500]"; return false; }
        // Output limits depend on the PID usage (angle -> speed dps, speed -> effort %)
        // Keep them fairly wide but check min <= max
        if (pid.pid_output_max < pid.pid_output_min) { error = name + ".output_max < output_min"; return false; }
        if (pid.pid_iterm_max < pid.pid_iterm_min) { error = name + ".iterm_max < iterm_min"; return false; }
        return true;
    };
    if (!validatePID(config.pid_angle, "pid_angle")) return false;
    if (!validatePID(config.pid_speed_left, "pid_speed_left")) return false;
    if (!validatePID(config.pid_speed_right, "pid_speed_right")) return false;
    if (!validatePID(config.pid_yaw_rate, "pid_yaw_rate")) return false;

    // SystemBehaviorConfig
    if (config.behavior.joystick_deadzone < 0.0f || config.behavior.joystick_deadzone > 1.0f) { error = "behavior.joystick_deadzone [0,1]"; return false; }
    if (config.behavior.joystick_timeout_ms < 1 || config.behavior.joystick_timeout_ms > 10000) { error = "behavior.joystick_timeout_ms [1,10k]"; return false; }
    if (config.behavior.joystick_check_interval_ms < 1 || config.behavior.joystick_check_interval_ms > 10000) { error = "behavior.joystick_check_interval_ms [1,10k]"; return false; }
    if (config.behavior.max_target_angular_velocity_dps <= 0.0f || config.behavior.max_target_angular_velocity_dps > 1000.0f) { error = "behavior.max_target_angular_velocity_dps (>0, <=1k)"; return false; }
    if (config.behavior.fall_pitch_threshold_deg < 10.0f || config.behavior.fall_pitch_threshold_deg > 90.0f) { error = "behavior.fall_pitch_threshold_deg [10,90]"; return false; }
    if (config.behavior.fall_threshold_duration_ms < 1 || config.behavior.fall_threshold_duration_ms > 10000) { error = "behavior.fall_threshold_duration_ms [1,10k]"; return false; }
    if (config.behavior.recovery_pitch_threshold_deg < 0.0f || config.behavior.recovery_pitch_threshold_deg > 30.0f) { error = "behavior.recovery_pitch_threshold_deg [0,30]"; return false; }
    if (config.behavior.recovery_hold_duration_ms < 1 || config.behavior.recovery_hold_duration_ms > 60000) { error = "behavior.recovery_hold_duration_ms [1,60k]"; return false; }
    if (config.behavior.imu_recovery_max_attempts < 1 || config.behavior.imu_recovery_max_attempts > 10) { error = "behavior.imu_recovery_max_attempts [1,10]"; return false; }
    if (config.behavior.imu_recovery_delay_ms < 1 || config.behavior.imu_recovery_delay_ms > 60000) { error = "behavior.imu_recovery_delay_ms [1,60k]"; return false; }
    if (config.behavior.battery_oversampling_count < 1 || config.behavior.battery_oversampling_count > 1024) { error = "behavior.battery_oversampling_count [1,1024]"; return false; }
    if (config.behavior.battery_read_interval_ms < 100 || config.behavior.battery_read_interval_ms > 60000) { error = "behavior.battery_read_interval_ms [100,60k]"; return false; }
    if (config.behavior.imu_health_i2c_fail_threshold < 1 || config.behavior.imu_health_i2c_fail_threshold > 100) { error = "behavior.imu_health_i2c_fail_threshold [1,100]"; return false; }
    if (config.behavior.imu_health_no_data_threshold < 1 || config.behavior.imu_health_no_data_threshold > 100) { error = "behavior.imu_health_no_data_threshold [1,100]"; return false; }
    if (config.behavior.imu_health_data_timeout_ms < 1 || config.behavior.imu_health_data_timeout_ms > 10000) { error = "behavior.imu_health_data_timeout_ms [1,10k]"; return false; }
    if (config.behavior.imu_health_proactive_check_ms < 1 || config.behavior.imu_health_proactive_check_ms > 60000) { error = "behavior.imu_health_proactive_check_ms [1,60k]"; return false; }
    // RobotDimensionsConfig
    if (config.dimensions.wheelbase_m <= 0.01f || config.dimensions.wheelbase_m > 1.0f) { error = "dimensions.wheelbase_m [0.01, 1.0]"; return false; }
    // WebServerConfig
    if (config.web.telemetry_buffer_size < 1 || config.web.telemetry_buffer_size > 10000) { error = "web.telemetry_buffer_size [1,10k]"; return false; }
    if (config.web.max_config_post_size < 512 || config.web.max_config_post_size > 65536) { error = "web.max_config_post_size [512, 64k]"; return false; }

    return true; // All checks passed
}

esp_err_t ConfigurationService::getJsonString(std::string& jsonOutput) const {
    std::lock_guard<std::mutex> lock(m_mutex); // Lock for reading m_configData
    esp_err_t ret = m_configParser.serialize(m_configData, jsonOutput);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize current configuration.");
    }
    return ret;
}

esp_err_t ConfigurationService::saveInternal() {
    // Assumes mutex is held
    ESP_LOGD(TAG, "Saving configuration to storage key '%s'", m_configKey.c_str());
    std::string rawData;
    esp_err_t ret = m_configParser.serialize(m_configData, rawData);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize configuration for saving.");
        return ret;
    }
    ret = m_storageService.saveData(m_configKey, rawData);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save configuration data to storage (%s).", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Configuration saved successfully.");
    }
    return ret;
}