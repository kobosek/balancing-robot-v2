#include "JsonConfigParser.hpp"
#include "JsonConfigSectionCodecs.hpp"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_err.h"
#include <string>
#include <memory>
#include <vector>

namespace {
constexpr const char* TAG = "JsonConfigParser";
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
    ADD_SECTION("control", json_config_sections::serializeControl(config.control));
    ADD_SECTION("imu", json_config_sections::serializeImu(config.imu));
    ADD_SECTION("encoder", json_config_sections::serializeEncoder(config.encoder));
    ADD_SECTION("motor", json_config_sections::serializeMotor(config.motor));
    ADD_SECTION("battery", json_config_sections::serializeBattery(config.battery));
    ADD_SECTION("behavior", json_config_sections::serializeBehavior(config.behavior));
    ADD_SECTION("dimensions", json_config_sections::serializeDimensions(config.dimensions));
    ADD_SECTION("web", json_config_sections::serializeWeb(config.web));

    // --- PIDs ---
    ADD_SECTION("pid_angle", json_config_sections::serializePid(config.pid_angle));
    ADD_SECTION("pid_speed_left", json_config_sections::serializePid(config.pid_speed_left));
    ADD_SECTION("pid_speed_right", json_config_sections::serializePid(config.pid_speed_right));
    ADD_SECTION("pid_yaw_angle", json_config_sections::serializePid(config.pid_yaw_angle));
    ADD_SECTION("pid_yaw_rate", json_config_sections::serializePid(config.pid_yaw_rate));
    ADD_SECTION("pid_tuning", json_config_sections::serializePidTuning(config.pid_tuning));

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
        json_config_sections::deserializeWiFi(wifi_section, tempConfig.wifi);
    } else { ESP_LOGW(TAG, "'wifi' section missing."); }

    // --- Main Loop ---
    cJSON *main_loop_section = cJSON_GetObjectItem(root, "mainLoop");
    if (main_loop_section) {
        json_config_sections::deserializeMainLoop(main_loop_section, tempConfig.mainLoop);
    } else { ESP_LOGW(TAG, "'mainLoop' section missing."); }

    // --- Control ---
    cJSON *control_section = cJSON_GetObjectItem(root, "control");
    if (control_section) {
        if (!json_config_sections::deserializeControl(control_section, tempConfig.control)) control_success = false;
    } else { ESP_LOGW(TAG, "'control' section missing."); control_success = false; }

    // --- IMU ---
    cJSON *imu_section = cJSON_GetObjectItem(root, "imu");
    if (imu_section) {
        json_config_sections::deserializeImu(imu_section, tempConfig.imu);
    } else { ESP_LOGW(TAG, "'imu' section missing."); }

    // --- Encoder ---
    cJSON *encoder_section = cJSON_GetObjectItem(root, "encoder");
    if (encoder_section) {
        json_config_sections::deserializeEncoder(encoder_section, tempConfig.encoder);
    } else { ESP_LOGW(TAG, "'encoder' section missing."); }

    // --- Motor ---
    cJSON *motor_section = cJSON_GetObjectItem(root, "motor");
    if (motor_section) {
        json_config_sections::deserializeMotor(motor_section, tempConfig.motor);
    } else { ESP_LOGW(TAG, "'motor' section missing."); }

     // --- Battery ---
    cJSON *battery_section = cJSON_GetObjectItem(root, "battery");
    if (battery_section) {
        json_config_sections::deserializeBattery(battery_section, tempConfig.battery);
    } else { ESP_LOGW(TAG, "'battery' section missing."); }

    // --- System Behavior ---
    cJSON *behavior_section = cJSON_GetObjectItem(root, "behavior");
    if (behavior_section) {
        if (!json_config_sections::deserializeBehavior(behavior_section, tempConfig.behavior)) behavior_success = false;
    } else { ESP_LOGW(TAG, "'behavior' section missing."); behavior_success = false; }
    // --- Robot Dimensions ---
    cJSON *dimensions_section = cJSON_GetObjectItem(root, "dimensions");
    if (dimensions_section) {
        if (!json_config_sections::deserializeDimensions(dimensions_section, tempConfig.dimensions)) dimensions_success = false;
    } else { ESP_LOGW(TAG, "'dimensions' section missing."); dimensions_success = false; }
    // --- Web Server ---
    cJSON *web_section = cJSON_GetObjectItem(root, "web");
    if (web_section) {
        if (!json_config_sections::deserializeWeb(web_section, tempConfig.web)) web_success = false;
    } else { ESP_LOGW(TAG, "'web' section missing."); web_success = false; }

    // --- PIDs ---
    cJSON *pid_section;
    pid_section = cJSON_GetObjectItem(root, "pid_angle");
    if (pid_section && !json_config_sections::deserializePid(pid_section, tempConfig.pid_angle)) pid_success = false; else if (!pid_section) { ESP_LOGW(TAG, "'pid_angle' missing."); pid_success = false; }

    pid_section = cJSON_GetObjectItem(root, "pid_speed_left");
    if (pid_section && !json_config_sections::deserializePid(pid_section, tempConfig.pid_speed_left)) pid_success = false; else if (!pid_section) { ESP_LOGW(TAG, "'pid_speed_left' missing."); pid_success = false; }

    pid_section = cJSON_GetObjectItem(root, "pid_speed_right");
    if (pid_section && !json_config_sections::deserializePid(pid_section, tempConfig.pid_speed_right)) pid_success = false; else if (!pid_section) { ESP_LOGW(TAG, "'pid_speed_right' missing."); pid_success = false; }

    pid_section = cJSON_GetObjectItem(root, "pid_yaw_angle");
    if (pid_section) {
        if (!json_config_sections::deserializePid(pid_section, tempConfig.pid_yaw_angle)) pid_success = false;
    } else {
        ESP_LOGW(TAG, "'pid_yaw_angle' missing. Using defaults.");
    }

    pid_section = cJSON_GetObjectItem(root, "pid_yaw_rate");
    if (pid_section && !json_config_sections::deserializePid(pid_section, tempConfig.pid_yaw_rate)) pid_success = false; else if (!pid_section) { ESP_LOGW(TAG, "'pid_yaw_rate' missing."); pid_success = false; }

    cJSON *pid_tuning_section = cJSON_GetObjectItem(root, "pid_tuning");
    if (pid_tuning_section) {
        json_config_sections::deserializePidTuning(pid_tuning_section, tempConfig.pid_tuning);
    } else {
        ESP_LOGW(TAG, "'pid_tuning' missing. Using defaults.");
    }
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
