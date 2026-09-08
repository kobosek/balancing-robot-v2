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
    cJSON_SetNumberValue(cJSON_GetObjectItem(root, "config_version"), 1);
    cJSON* behavior = cJSON_GetObjectItem(root, "behavior");
    cJSON_DeleteItemFromObject(behavior, "imu_max_sample_age_ms");
    cJSON_DeleteItemFromObject(behavior, "imu_reconnect_interval_ms");
    cJSON_AddNumberToObject(behavior, "imu_health_data_timeout_ms", 500);
    char* legacy = cJSON_PrintUnformatted(root);
    ConfigData migrated;
    TEST_ASSERT_EQUAL(ESP_OK, parser.deserialize(legacy, migrated));
    cJSON_free(legacy); cJSON_Delete(root);
    TEST_ASSERT_EQUAL_INT(2, migrated.config_version);
    TEST_ASSERT_EQUAL_INT(20, migrated.behavior.imu_max_sample_age_ms);
    TEST_ASSERT_EQUAL_INT(1000, migrated.behavior.imu_reconnect_interval_ms);
    TEST_ASSERT_EQUAL_STRING(source.wifi.password.c_str(), migrated.wifi.password.c_str());
    TEST_ASSERT_EQUAL_FLOAT(1.25f, migrated.imu.gyro_offset_y);
    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(migrated, json));
    TEST_ASSERT_EQUAL(std::string::npos, json.find("imu_health"));
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
