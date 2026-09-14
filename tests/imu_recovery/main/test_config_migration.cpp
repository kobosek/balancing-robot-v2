#include "unity.h"
#include "JsonConfigParser.hpp"
#include "ConfigValidator.hpp"
#include "MPU6050Profile.hpp"
#include "cJSON.h"
TEST_CASE("legacy migration preserves unrelated values and ignores old health timeout", "[imu][config]") {
    ConfigData source; source.wifi.ssid = "test-network"; source.wifi.password = "test-only-value";
    source.imu.gyro_offset_y = 1.25f;
    JsonConfigParser parser;
    std::string json; TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(source, json));
    cJSON* root = cJSON_Parse(json.c_str());
    cJSON* control = cJSON_GetObjectItem(root, "control");
    cJSON* strategies = cJSON_GetObjectItem(control, "strategies");
    cJSON* nested = cJSON_GetObjectItem(strategies, "nested_pid");
    const char* legacyPidNames[] = {"angle", "speed_left", "speed_right", "yaw_angle", "yaw_rate"};
    const char* legacySectionNames[] = {"pid_angle", "pid_speed_left", "pid_speed_right", "pid_yaw_angle", "pid_yaw_rate"};
    for (size_t i = 0; i < 5; ++i) {
        cJSON* legacyPid = cJSON_DetachItemFromObject(nested, legacyPidNames[i]);
        TEST_ASSERT_NOT_NULL(legacyPid);
        cJSON_AddItemToObject(root, legacySectionNames[i], legacyPid);
    }
    cJSON_DeleteItemFromObject(control, "strategies");
    cJSON_DeleteItemFromObject(control, "balance_strategy");
    cJSON_DeleteItemFromObject(control, "strategies_revision");
    cJSON_SetNumberValue(cJSON_GetObjectItem(root, "config_version"), 1);
    cJSON* behavior = cJSON_GetObjectItem(root, "behavior");
    cJSON_DeleteItemFromObject(behavior, "imu_max_sample_age_ms");
    cJSON_DeleteItemFromObject(behavior, "imu_reconnect_interval_ms");
    cJSON_AddNumberToObject(behavior, "imu_health_data_timeout_ms", 500);
    char* legacy = cJSON_PrintUnformatted(root);
    ConfigData migrated;
    TEST_ASSERT_EQUAL(ESP_OK, parser.deserialize(legacy, migrated));
    cJSON_free(legacy); cJSON_Delete(root);
    TEST_ASSERT_EQUAL_INT(3, migrated.config_version);
    TEST_ASSERT_EQUAL_INT(20, migrated.behavior.imu_max_sample_age_ms);
    TEST_ASSERT_EQUAL_INT(1000, migrated.behavior.imu_reconnect_interval_ms);
    TEST_ASSERT_EQUAL_STRING(source.wifi.password.c_str(), migrated.wifi.password.c_str());
    TEST_ASSERT_EQUAL_FLOAT(1.25f, migrated.imu.gyro_offset_y);
    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(migrated, json));
    TEST_ASSERT_EQUAL(std::string::npos, json.find("imu_health"));
}
TEST_CASE("version three keeps strategy sets separate and rejects incomplete activation", "[config][strategy]") {
    JsonConfigParser parser;
    ConfigData source;
    source.wifi.ssid = "test-network";
    source.control.strategies.nested_pid.speed_left.pid_kp = 0.11f;
    source.control.strategies.longitudinal_cascade.velocity.pid_kp = 1.7f;
    source.control.strategies.longitudinal_cascade.configured = false;
    source.control.strategies.longitudinal_cascade.loop_mode = LongitudinalLoopMode::VELOCITY;
    source.control.strategies.longitudinal_cascade.hold_enter_velocity_mps = 0.03f;
    source.control.strategies.longitudinal_cascade.hold_exit_velocity_mps = 0.07f;
    source.control.strategies.longitudinal_cascade.hold_pitch_error_deadband_deg = 1.5f;
    source.control.strategies.longitudinal_cascade.hold_pitch_rate_deadband_dps = 12.0f;
    source.control.strategies.longitudinal_cascade.hold_settle_time_ms = 240;
    source.control.strategies.longitudinal_cascade.sync_enabled = true;
    source.control.strategies.longitudinal_cascade.sync_kp = 0.4f;
    source.control.strategies.longitudinal_cascade.sync_kd = 0.2f;
    source.control.strategies.longitudinal_cascade.sync_position_deadband_m = 0.003f;
    source.control.strategies.longitudinal_cascade.sync_velocity_deadband_mps = 0.012f;
    source.control.strategies.longitudinal_cascade.velocity_to_pitch_sign = -1;
    source.control.strategies.longitudinal_cascade.pitch_to_effort_sign = -1;
    source.control.strategies.nested_pid.max_target_angular_velocity_dps = 123.0f;
    source.behavior.max_target_angular_velocity_dps = 123.0f;
    source.control.strategies.longitudinal_cascade.motion_request_limit_enabled = false;
    source.control.strategies.longitudinal_cascade.motion_request_limit_pitch_start_deg = 5.0f;
    source.control.strategies.longitudinal_cascade.motion_request_limit_pitch_full_deg = 9.0f;
    source.control.strategies.longitudinal_cascade.motion_request_limit_pitch_release_deg = 4.0f;
    source.control.strategies.longitudinal_cascade.motion_request_limit_effort_start = 0.7f;
    source.control.strategies.longitudinal_cascade.motion_request_limit_effort_full = 0.9f;
    source.control.strategies.longitudinal_cascade.motion_request_limit_effort_release = 0.6f;
    source.control.strategies.longitudinal_cascade.motion_request_limit_min_scale = 0.25f;
    source.config_revision = 17;
    std::string json;
    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(source, json));

    ConfigData roundTrip;
    TEST_ASSERT_EQUAL(ESP_OK, parser.deserialize(json, roundTrip));
    TEST_ASSERT_EQUAL_FLOAT(0.11f, roundTrip.control.strategies.nested_pid.speed_left.pid_kp);
    TEST_ASSERT_EQUAL_FLOAT(1.7f, roundTrip.control.strategies.longitudinal_cascade.velocity.pid_kp);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LongitudinalLoopMode::VELOCITY),
                          static_cast<int>(roundTrip.control.strategies.longitudinal_cascade.loop_mode));
    TEST_ASSERT_EQUAL_FLOAT(0.03f,
                            roundTrip.control.strategies.longitudinal_cascade.hold_enter_velocity_mps);
    TEST_ASSERT_EQUAL_FLOAT(0.07f,
                            roundTrip.control.strategies.longitudinal_cascade.hold_exit_velocity_mps);
    TEST_ASSERT_EQUAL_FLOAT(1.5f,
                            roundTrip.control.strategies.longitudinal_cascade.hold_pitch_error_deadband_deg);
    TEST_ASSERT_EQUAL_UINT32(240,
                             roundTrip.control.strategies.longitudinal_cascade.hold_settle_time_ms);
    TEST_ASSERT_FALSE(roundTrip.control.strategies.longitudinal_cascade.motion_request_limit_enabled);
    TEST_ASSERT_EQUAL_FLOAT(5.0f,
                            roundTrip.control.strategies.longitudinal_cascade.motion_request_limit_pitch_start_deg);
    TEST_ASSERT_EQUAL_FLOAT(9.0f,
                            roundTrip.control.strategies.longitudinal_cascade.motion_request_limit_pitch_full_deg);
    TEST_ASSERT_EQUAL_FLOAT(0.25f,
                            roundTrip.control.strategies.longitudinal_cascade.motion_request_limit_min_scale);
    TEST_ASSERT_TRUE(roundTrip.control.strategies.longitudinal_cascade.sync_enabled);
    TEST_ASSERT_EQUAL_FLOAT(0.4f,
                            roundTrip.control.strategies.longitudinal_cascade.sync_kp);
    TEST_ASSERT_EQUAL_FLOAT(0.2f,
                            roundTrip.control.strategies.longitudinal_cascade.sync_kd);
    TEST_ASSERT_EQUAL_FLOAT(0.003f,
                            roundTrip.control.strategies.longitudinal_cascade.sync_position_deadband_m);
    TEST_ASSERT_EQUAL_FLOAT(0.012f,
                            roundTrip.control.strategies.longitudinal_cascade.sync_velocity_deadband_mps);
    TEST_ASSERT_EQUAL_INT(-1,
                          roundTrip.control.strategies.longitudinal_cascade.velocity_to_pitch_sign);
    TEST_ASSERT_EQUAL_INT(-1,
                          roundTrip.control.strategies.longitudinal_cascade.pitch_to_effort_sign);
    TEST_ASSERT_EQUAL_FLOAT(123.0f,
                            roundTrip.control.strategies.nested_pid.max_target_angular_velocity_dps);
    TEST_ASSERT_EQUAL_FLOAT(123.0f,
                            roundTrip.behavior.max_target_angular_velocity_dps);
    TEST_ASSERT_EQUAL_UINT32(17, roundTrip.config_revision);
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(roundTrip.control.strategies.active));

    cJSON* root = cJSON_Parse(json.c_str());
    cJSON* control = cJSON_GetObjectItem(root, "control");
    cJSON_ReplaceItemInObject(control, "balance_strategy", cJSON_CreateString("longitudinal_cascade"));
    cJSON* strategies = cJSON_GetObjectItem(control, "strategies");
    cJSON_ReplaceItemInObject(strategies, "active", cJSON_CreateString("longitudinal_cascade"));
    char* unavailable = cJSON_PrintUnformatted(root);
    ConfigData output;
    TEST_ASSERT_EQUAL(ESP_OK, parser.deserialize(unavailable, output));
    ConfigValidator validator;
    std::string validationError;
    TEST_ASSERT_FALSE(validator.validate(output, validationError));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, validationError.find("requires configured"));
    cJSON_free(unavailable);
    cJSON_Delete(root);
}
TEST_CASE("legacy behavior yaw limit migrates to the NestedPid strategy record", "[config][strategy]") {
    JsonConfigParser parser;
    ConfigData source;
    source.wifi.ssid = "test-network";
    std::string json;
    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(source, json));
    cJSON* root = cJSON_Parse(json.c_str());
    cJSON* control = cJSON_GetObjectItem(root, "control");
    cJSON* strategies = cJSON_GetObjectItem(control, "strategies");
    cJSON* nested = cJSON_GetObjectItem(strategies, "nested_pid");
    cJSON_DeleteItemFromObject(nested, "max_target_angular_velocity_dps");
    cJSON* behavior = cJSON_GetObjectItem(root, "behavior");
    cJSON_ReplaceItemInObject(behavior, "max_target_angular_velocity_dps",
                              cJSON_CreateNumber(87.0));
    char* legacy = cJSON_PrintUnformatted(root);
    ConfigData migrated;
    TEST_ASSERT_EQUAL(ESP_OK, parser.deserialize(legacy, migrated));
    TEST_ASSERT_EQUAL_FLOAT(87.0f,
                            migrated.control.strategies.nested_pid.max_target_angular_velocity_dps);
    TEST_ASSERT_EQUAL_FLOAT(87.0f, migrated.behavior.max_target_angular_velocity_dps);
    cJSON_free(legacy);
    cJSON_Delete(root);
}
TEST_CASE("canonical nested strategy fields reject conflicting compatibility aliases", "[config][strategy]") {
    JsonConfigParser parser;
    ConfigData source;
    source.wifi.ssid = "test-network";
    std::string json;
    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(source, json));
    cJSON* root = cJSON_Parse(json.c_str());
    cJSON* control = cJSON_GetObjectItem(root, "control");
    cJSON_ReplaceItemInObject(control, "max_target_pitch_offset_deg", cJSON_CreateNumber(9.0));
    char* conflicting = cJSON_PrintUnformatted(root);
    ConfigData output;
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parser.deserialize(conflicting, output));
    cJSON_free(conflicting);
    cJSON_Delete(root);

    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(source, json));
    root = cJSON_Parse(json.c_str());
    cJSON* behavior = cJSON_GetObjectItem(root, "behavior");
    cJSON_ReplaceItemInObject(behavior, "max_target_angular_velocity_dps",
                              cJSON_CreateNumber(99.0));
    conflicting = cJSON_PrintUnformatted(root);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parser.deserialize(conflicting, output));
    cJSON_free(conflicting);
    cJSON_Delete(root);
}

TEST_CASE("direction fields round-trip and reject non-unit signs", "[config][strategy]") {
    JsonConfigParser parser;
    ConfigData source;
    source.wifi.ssid = "test-network";
    auto& longitudinal = source.control.strategies.longitudinal_cascade;
    longitudinal.left_encoder_forward_sign = -1;
    longitudinal.right_encoder_forward_sign = 1;
    longitudinal.left_output_sign = -1;
    longitudinal.right_output_sign = -1;
    longitudinal.velocity_to_pitch_sign = -1;
    longitudinal.pitch_to_effort_sign = -1;
    std::string json;
    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(source, json));
    ConfigData roundTrip;
    TEST_ASSERT_EQUAL(ESP_OK, parser.deserialize(json, roundTrip));
    TEST_ASSERT_EQUAL_INT(-1, roundTrip.control.strategies.longitudinal_cascade.left_encoder_forward_sign);
    TEST_ASSERT_EQUAL_INT(1, roundTrip.control.strategies.longitudinal_cascade.right_encoder_forward_sign);
    TEST_ASSERT_EQUAL_INT(-1, roundTrip.control.strategies.longitudinal_cascade.left_output_sign);
    TEST_ASSERT_EQUAL_INT(-1, roundTrip.control.strategies.longitudinal_cascade.right_output_sign);
    TEST_ASSERT_EQUAL_INT(-1, roundTrip.control.strategies.longitudinal_cascade.velocity_to_pitch_sign);
    TEST_ASSERT_EQUAL_INT(-1, roundTrip.control.strategies.longitudinal_cascade.pitch_to_effort_sign);

    cJSON* root = cJSON_Parse(json.c_str());
    cJSON* control = cJSON_GetObjectItem(root, "control");
    cJSON* strategies = cJSON_GetObjectItem(control, "strategies");
    cJSON* longitudinalObject = cJSON_GetObjectItem(strategies, "longitudinal_cascade");
    cJSON_ReplaceItemInObject(longitudinalObject, "left_output_sign", cJSON_CreateNumber(0.0));
    char* invalid = cJSON_PrintUnformatted(root);
    ConfigData output;
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parser.deserialize(invalid, output));
    cJSON_free(invalid);
    cJSON_Delete(root);
}
TEST_CASE("v3 files without loop mode keep the safe baseline or recover a complete velocity setup", "[config][strategy]") {
    JsonConfigParser parser;
    ConfigData source;
    source.wifi.ssid = "test-network";
    auto& longitudinal = source.control.strategies.longitudinal_cascade;
    longitudinal.configured = true;
    longitudinal.velocity.pid_kp = 1.0f;
    longitudinal.max_velocity_mps = 1.0f;
    longitudinal.max_acceleration_mps2 = 1.0f;
    longitudinal.max_deceleration_mps2 = 1.0f;
    longitudinal.loop_mode = LongitudinalLoopMode::VELOCITY;

    std::string json;
    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(source, json));
    cJSON* root = cJSON_Parse(json.c_str());
    cJSON* control = cJSON_GetObjectItem(root, "control");
    cJSON* strategies = cJSON_GetObjectItem(control, "strategies");
    cJSON* oldLongitudinal = cJSON_GetObjectItem(strategies, "longitudinal_cascade");
    cJSON_DeleteItemFromObject(oldLongitudinal, "loop_mode");
    char* legacyVelocity = cJSON_PrintUnformatted(root);
    ConfigData migratedVelocity;
    TEST_ASSERT_EQUAL(ESP_OK, parser.deserialize(legacyVelocity, migratedVelocity));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LongitudinalLoopMode::VELOCITY),
                          static_cast<int>(migratedVelocity.control.strategies.longitudinal_cascade.loop_mode));
    cJSON_free(legacyVelocity);
    cJSON_Delete(root);

    longitudinal.configured = false;
    longitudinal.max_velocity_mps = 0.0f;
    longitudinal.max_acceleration_mps2 = 0.0f;
    longitudinal.max_deceleration_mps2 = 0.0f;
    longitudinal.velocity.pid_kp = 0.0f;
    longitudinal.loop_mode = LongitudinalLoopMode::PITCH_ONLY;
    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(source, json));
    root = cJSON_Parse(json.c_str());
    control = cJSON_GetObjectItem(root, "control");
    strategies = cJSON_GetObjectItem(control, "strategies");
    oldLongitudinal = cJSON_GetObjectItem(strategies, "longitudinal_cascade");
    cJSON_DeleteItemFromObject(oldLongitudinal, "loop_mode");
    char* legacyPitch = cJSON_PrintUnformatted(root);
    ConfigData migratedPitch;
    TEST_ASSERT_EQUAL(ESP_OK, parser.deserialize(legacyPitch, migratedPitch));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LongitudinalLoopMode::PITCH_ONLY),
                          static_cast<int>(migratedPitch.control.strategies.longitudinal_cascade.loop_mode));
    cJSON_free(legacyPitch);
    cJSON_Delete(root);
}
TEST_CASE("configured longitudinal pitch baseline requires usable limits and gains", "[config][strategy]") {
    ConfigData config;
    config.wifi.ssid = "test-network";
    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.configured = true;
    longitudinal.pitch = {0.2f, 0.0f, 0.01f, -1.0f, 1.0f, -1.0f, 1.0f};
    longitudinal.max_pitch_offset_deg = 5.0f;
    longitudinal.max_pitch_rate_dps = 90.0f;
    longitudinal.max_effort = 0.8f;

    ConfigValidator validator;
    std::string error;
    TEST_ASSERT_TRUE(validator.validate(config, error));

    longitudinal.max_effort = 0.0f;
    TEST_ASSERT_FALSE(validator.validate(config, error));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, error.find("requires pitch"));

    longitudinal.max_effort = 0.8f;
    config.control.strategies.active = BalanceStrategyId::LONGITUDINAL_CASCADE;
    TEST_ASSERT_TRUE(validator.validate(config, error));
}

TEST_CASE("position hold and synchronization activation require their typed limits", "[config][strategy]") {
    ConfigData config;
    config.wifi.ssid = "test-network";
    config.control.strategies.active = BalanceStrategyId::LONGITUDINAL_CASCADE;
    auto& longitudinal = config.control.strategies.longitudinal_cascade;
    longitudinal.configured = true;
    longitudinal.pitch = {0.2f, 0.0f, 0.01f, -1.0f, 1.0f, -1.0f, 1.0f};
    longitudinal.velocity = {1.0f, 1.0f, 0.0f, -2.0f, 2.0f, -2.0f, 2.0f};
    longitudinal.max_pitch_offset_deg = 5.0f;
    longitudinal.max_pitch_rate_dps = 90.0f;
    longitudinal.max_velocity_mps = 1.0f;
    longitudinal.max_hold_velocity_mps = 0.2f;
    longitudinal.max_acceleration_mps2 = 1.0f;
    longitudinal.max_deceleration_mps2 = 1.0f;
    longitudinal.position_kp = 1.0f;
    longitudinal.hold_position_deadband_m = 0.01f;
    longitudinal.loop_mode = LongitudinalLoopMode::POSITION_HOLD;
    longitudinal.max_effort = 0.8f;
    longitudinal.sync_enabled = true;
    longitudinal.sync_kp = 0.5f;
    longitudinal.sync_position_deadband_m = 0.002f;
    longitudinal.sync_velocity_deadband_mps = 0.005f;
    longitudinal.sync_max_effort = 0.2f;

    ConfigValidator validator;
    std::string error;
    TEST_ASSERT_TRUE(validator.validate(config, error));

    longitudinal.position_kp = 0.0f;
    TEST_ASSERT_FALSE(validator.validate(config, error));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, error.find("position_hold"));

    longitudinal.position_kp = 1.0f;
    longitudinal.sync_max_effort = 0.0f;
    TEST_ASSERT_FALSE(validator.validate(config, error));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, error.find("synchronization"));
}
TEST_CASE("motor deadzone validation is bounded by the LEDC duty resolution", "[config][motor]") {
    ConfigData config;
    config.wifi.ssid = "test-network";
    ConfigValidator validator;
    std::string error;

    config.motor.duty_resolution = 0;
    TEST_ASSERT_FALSE(validator.validate(config, error));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, error.find("duty_resolution"));

    config.motor.duty_resolution = 10;
    config.motor.deadzone_duty = 1024;
    TEST_ASSERT_FALSE(validator.validate(config, error));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, error.find("deadzone_duty"));

    config.motor.deadzone_duty = 500;
    TEST_ASSERT_TRUE(validator.validate(config, error));
}
TEST_CASE("future version and malformed new field are rejected without replacing output", "[imu][config]") {
    JsonConfigParser parser; ConfigData output; output.imu.gyro_offset_y = 9;
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, parser.deserialize("{\"config_version\":99}", output));
    TEST_ASSERT_EQUAL_FLOAT(9, output.imu.gyro_offset_y);
    std::string json; parser.serialize(ConfigData{}, json);
    cJSON* root = cJSON_Parse(json.c_str());
    cJSON* behavior = cJSON_GetObjectItem(root, "behavior");
    cJSON_ReplaceItemInObject(behavior, "imu_max_sample_age_ms", cJSON_CreateString("20"));
    char* malformed = cJSON_PrintUnformatted(root);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parser.deserialize(malformed, output));
    cJSON_free(malformed); cJSON_Delete(root);
}
TEST_CASE("transport budget rejects infeasible rate threshold and reserved DLPF", "[imu][config]") {
    MPU6050Config imu;
    TEST_ASSERT_TRUE(MPU6050Profile::timingValid(imu, 20, 5));
    imu.dlpf_config = 0;
    TEST_ASSERT_EQUAL_UINT32(500, MPU6050Profile::fromConfig(imu).samplePeriodUs);
    TEST_ASSERT_FALSE(MPU6050Profile::timingValid(imu, 20, 5));
    imu = {}; imu.fifo_read_threshold = 240;
    TEST_ASSERT_FALSE(MPU6050Profile::timingValid(imu, 20, 5));
    ConfigData config; config.wifi.ssid = "test-network"; config.imu.dlpf_config = 7;
    std::string error; ConfigValidator validator;
    TEST_ASSERT_FALSE(validator.validate(config, error));
}
