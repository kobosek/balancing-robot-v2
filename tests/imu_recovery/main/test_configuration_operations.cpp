#include "unity.h"

#include "ConfigurationService.hpp"
#include "ControlOperationGate.hpp"
#include "JsonConfigParser.hpp"
#include "SPIFFSStorageService.hpp"
#include "cJSON.h"

#include <cstdio>
#include <string>

namespace {

class MemoryStorage final : public IStorageService {
public:
    std::string config;
    std::string operation;
    bool freshStorage = true;
    int operationSaveCount = 0;
    int failConfigSaves = 0;
    int failOperationSaveAt = 0;

    esp_err_t init() override { return ESP_OK; }
    bool isFreshStorage() const override { return freshStorage; }

    esp_err_t loadData(const std::string& key, std::string& data) override
    {
        const std::string* source = nullptr;
        if (key == "config.json") {
            source = &config;
        } else if (key == "config.json.operation") {
            source = &operation;
        } else {
            return ESP_ERR_NOT_FOUND;
        }
        if (source->empty()) {
            return ESP_ERR_NOT_FOUND;
        }
        data = *source;
        return ESP_OK;
    }

    esp_err_t saveData(const std::string& key,
                       const std::string& data) override
    {
        if (key == "config.json.operation") {
            ++operationSaveCount;
            if (failOperationSaveAt != 0 &&
                operationSaveCount == failOperationSaveAt) {
                // One-shot failure leaves the previous complete journal
                // record available for a simulated restart.
                failOperationSaveAt = 0;
                return ESP_FAIL;
            }
            operation = data;
            return ESP_OK;
        }
        if (key == "config.json") {
            if (failConfigSaves > 0) {
                --failConfigSaves;
                return ESP_FAIL;
            }
            config = data;
            return ESP_OK;
        }
        return ESP_ERR_NOT_FOUND;
    }
};

ConfigData makeValidConfig(JsonConfigParser& parser, MemoryStorage& storage)
{
    ConfigData config;
    config.wifi.ssid = "test-network";
    config.wifi.password.clear();
    std::string serialized;
    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(config, serialized));
    storage.config = serialized;
    return config;
}

std::string withOperationId(JsonConfigParser& parser,
                            const ConfigData& config,
                            uint64_t operationId)
{
    std::string serialized;
    TEST_ASSERT_EQUAL(ESP_OK, parser.serialize(config, serialized));
    cJSON* root = cJSON_Parse(serialized.c_str());
    TEST_ASSERT_NOT_NULL(root);

    char operationText[24] = {};
    std::snprintf(operationText, sizeof(operationText), "%llu",
                  static_cast<unsigned long long>(operationId));
    TEST_ASSERT_NOT_NULL(cJSON_AddStringToObject(root, "operation_id",
                                                  operationText));
    char* printed = cJSON_PrintUnformatted(root);
    TEST_ASSERT_NOT_NULL(printed);
    std::string result(printed);
    cJSON_free(printed);
    cJSON_Delete(root);
    return result;
}

ConfigData readConfig(JsonConfigParser& parser, const MemoryStorage& storage)
{
    ConfigData config;
    TEST_ASSERT_EQUAL(ESP_OK, parser.deserialize(storage.config, config));
    return config;
}

} // namespace

TEST_CASE("configuration updates cannot race a motion reservation",
          "[config][operation][gate]")
{
    JsonConfigParser parser;
    MemoryStorage storage;
    makeValidConfig(parser, storage);
    ControlOperationGate gate;
    ConfigurationService service(storage, parser, EventBus::getInstance(),
                                 "config.json", &gate);
    TEST_ASSERT_EQUAL(ESP_OK, service.init());

    ConfigData candidate = service.getConfigData();
    candidate.control.joystick_exponent = 2.0f;
    const std::string request = withOperationId(parser, candidate, 101);

    ControlOperationReservation motion;
    TEST_ASSERT_TRUE(gate.tryAcquire(ControlOperationKind::MOTION, motion));
    std::string error;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      service.updateConfigFromJson(request, &error));
    TEST_ASSERT_EQUAL_UINT32(0, service.getConfigData().config_revision);
    TEST_ASSERT_TRUE(gate.release(motion));
}

TEST_CASE("missing config on an existing medium enters recovery",
          "[config][operation][recovery]")
{
    JsonConfigParser parser;
    MemoryStorage storage;
    storage.freshStorage = false;
    ControlOperationGate gate;
    ConfigurationService service(storage, parser, EventBus::getInstance(),
                                 "config.json", &gate);

    TEST_ASSERT_EQUAL(ESP_OK, service.init());
    TEST_ASSERT_TRUE(service.isOperationRecoveryPending());
    TEST_ASSERT_TRUE(gate.recoveryPending());
    TEST_ASSERT_FALSE(service.getOperationStatus().known);
    TEST_ASSERT_EQUAL_UINT32(0, service.getConfigData().config_revision);
}

TEST_CASE("missing config on fresh storage keeps first boot defaults path",
          "[config][operation][first_boot]")
{
    JsonConfigParser parser;
    MemoryStorage storage;
    storage.freshStorage = true;
    ControlOperationGate gate;
    ConfigurationService service(storage, parser, EventBus::getInstance(),
                                 "config.json", &gate);

    TEST_ASSERT_EQUAL(ESP_OK, service.init());
    TEST_ASSERT_FALSE(service.isOperationRecoveryPending());
    TEST_ASSERT_FALSE(gate.recoveryPending());
    TEST_ASSERT_EQUAL_UINT32(0, service.getConfigData().config_revision);
}

TEST_CASE("configuration storage failure rolls back the runtime snapshot",
          "[config][operation][fault]")
{
    JsonConfigParser parser;
    MemoryStorage storage;
    makeValidConfig(parser, storage);
    ControlOperationGate gate;
    ConfigurationService service(storage, parser, EventBus::getInstance(),
                                 "config.json", &gate);
    TEST_ASSERT_EQUAL(ESP_OK, service.init());

    ConfigData candidate = service.getConfigData();
    candidate.control.joystick_exponent = 2.25f;
    storage.failConfigSaves = 1;
    std::string error;
    const esp_err_t result = service.updateConfigFromJson(
        withOperationId(parser, candidate, 202), &error);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, result);
    TEST_ASSERT_EQUAL_UINT32(0, service.getConfigData().config_revision);
    TEST_ASSERT_EQUAL_FLOAT(1.5f,
                            service.getConfigData().control.joystick_exponent);
    const ConfigData persisted = readConfig(parser, storage);
    TEST_ASSERT_EQUAL_UINT32(0, persisted.config_revision);
    TEST_ASSERT_EQUAL_FLOAT(1.5f, persisted.control.joystick_exponent);
    TEST_ASSERT_FALSE(gate.isHeld());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlOperationPhase::FAILED),
                          static_cast<int>(service.getOperationStatus().phase));
}

TEST_CASE("invalid persisted config blocks motion until an explicit repair POST",
          "[config][operation][recovery]")
{
    JsonConfigParser parser;
    MemoryStorage storage;
    storage.config = "{ this is not a configuration document";
    ControlOperationGate gate;
    ConfigurationService service(storage, parser, EventBus::getInstance(),
                                 "config.json", &gate);

    TEST_ASSERT_EQUAL(ESP_OK, service.init());
    TEST_ASSERT_TRUE(service.isOperationRecoveryPending());
    TEST_ASSERT_TRUE(gate.recoveryPending());
    TEST_ASSERT_FALSE(service.getOperationStatus().known);

    // A new, complete v3 document is the repair authority when the old
    // document/journal has no trustworthy operation identity. It still goes
    // through the normal revision and storage transaction before the latch
    // is cleared.
    ConfigData repaired = service.getConfigData();
    repaired.wifi.ssid = "repaired-network";
    repaired.config_revision = 1;
    std::string error;
    TEST_ASSERT_EQUAL(ESP_OK, service.updateConfigFromJson(
        withOperationId(parser, repaired, 404), &error));
    TEST_ASSERT_FALSE(service.isOperationRecoveryPending());
    TEST_ASSERT_FALSE(gate.recoveryPending());
    TEST_ASSERT_EQUAL_UINT32(1, service.getConfigData().config_revision);

    ConfigData persisted = readConfig(parser, storage);
    TEST_ASSERT_EQUAL_STRING("repaired-network", persisted.wifi.ssid.c_str());
}

TEST_CASE("invalid main config cannot be dismissed through a stale journal",
          "[config][operation][recovery]")
{
    JsonConfigParser parser;
    MemoryStorage storage;
    storage.config = "{ invalid main document";
    // This journal looks like an interrupted operation at the in-memory
    // default revision.  It must not authorize acknowledging the defaults
    // when the main configuration itself is unreadable.
    storage.operation = R"({"version":1,"operation_id":"505",
        "fingerprint":"123","base_revision":0,"target_revision":1,
        "phase":"preparing","result_code":0})";
    ControlOperationGate gate;
    ConfigurationService service(storage, parser, EventBus::getInstance(),
                                 "config.json", &gate);

    TEST_ASSERT_EQUAL(ESP_OK, service.init());
    TEST_ASSERT_TRUE(service.isOperationRecoveryPending());
    TEST_ASSERT_TRUE(gate.recoveryPending());
    TEST_ASSERT_FALSE(service.getOperationStatus().known);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, service.acknowledgeOperation(505));

    ConfigData repaired = service.getConfigData();
    repaired.wifi.ssid = "repaired-after-journal";
    repaired.config_revision = 1;
    std::string error;
    TEST_ASSERT_EQUAL(ESP_OK, service.updateConfigFromJson(
        withOperationId(parser, repaired, 506), &error));
    TEST_ASSERT_FALSE(service.isOperationRecoveryPending());
    TEST_ASSERT_FALSE(gate.recoveryPending());
}

TEST_CASE("terminal journal failure is reported and reconciled after restart",
          "[config][operation][recovery]")
{
    JsonConfigParser parser;
    MemoryStorage storage;
    makeValidConfig(parser, storage);
    ControlOperationGate firstGate;
    ConfigurationService first(storage, parser, EventBus::getInstance(),
                               "config.json", &firstGate);
    TEST_ASSERT_EQUAL(ESP_OK, first.init());

    ConfigData candidate = first.getConfigData();
    candidate.control.joystick_exponent = 2.5f;
    // PREPARING, WRITING, APPLYING and the terminal journal write are the
    // operation records around one successful document write.  Fail the
    // terminal write so the APPLYING record survives a simulated reset.
    storage.failOperationSaveAt = 4;
    std::string error;
    const esp_err_t result = first.updateConfigFromJson(
        withOperationId(parser, candidate, 303), &error);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, result);
    TEST_ASSERT_TRUE(first.isOperationRecoveryPending());
    TEST_ASSERT_EQUAL_UINT32(1, first.getConfigData().config_revision);
    TEST_ASSERT_FALSE(firstGate.isHeld());

    ControlOperationGate secondGate;
    ConfigurationService second(storage, parser, EventBus::getInstance(),
                                "config.json", &secondGate);
    TEST_ASSERT_EQUAL(ESP_OK, second.init());
    TEST_ASSERT_TRUE(second.isOperationRecoveryPending());
    TEST_ASSERT_TRUE(secondGate.recoveryPending());
    TEST_ASSERT_EQUAL(ESP_OK, second.acknowledgeOperation(303));
    TEST_ASSERT_FALSE(second.isOperationRecoveryPending());
    TEST_ASSERT_FALSE(secondGate.recoveryPending());
    const auto status = second.getOperationStatus();
    TEST_ASSERT_EQUAL_UINT64(303, status.operationId);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlOperationPhase::SUCCEEDED),
                          static_cast<int>(status.phase));

    ControlOperationReservation motion;
    TEST_ASSERT_TRUE(secondGate.tryAcquire(ControlOperationKind::MOTION,
                                           motion));
    TEST_ASSERT_TRUE(secondGate.release(motion));
}
