#include "include/ComponentHandler.hpp"
#include "include/WebServer.hpp"
#include "include/WifiManager.hpp"
#include "include/PIDController.hpp"
#include "include/MotorDriver.hpp"
#include "include/MPU6050Manager.hpp"
#include "include/Encoder.hpp"

#include <algorithm>
#include "esp_log.h"

esp_err_t ComponentHandler::init() {
    ESP_LOGI(TAG, "Initializing ComponentHandler");

    wifiManager = std::make_shared<WiFiManager>();
    esp_err_t ret = wifiManager->init(runtimeConfig);
    webServer = std::make_shared<WebServer>(*this, runtimeConfig);
    if (ret == ESP_OK) {
        webServer->init(runtimeConfig);
        registerObserver(wifiManager);
        registerObserver(webServer);
    }

    motorLeft = std::make_shared<MX1616H>(GPIO_NUM_5, GPIO_NUM_6, LEDC_CHANNEL_2, LEDC_CHANNEL_3);
    ret = motorLeft->init(runtimeConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MotorDriver");
        return ret;
    }

    motorRight = std::make_shared<MX1616H>(GPIO_NUM_4, GPIO_NUM_3, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
    ret = motorRight->init(runtimeConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MotorDriver");
        return ret;
    }

    encoderLeft = std::make_shared<PCNTEncoder>(GPIO_NUM_11, GPIO_NUM_10);

    ret = encoderLeft->init(runtimeConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Encoder");
        return ret;
    }

    encoderRight = std::make_shared<PCNTEncoder>(GPIO_NUM_12, GPIO_NUM_13);

    ret = encoderRight->init(runtimeConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Encoder");
        return ret;
    }

    mpu6050Manager = std::make_shared<MPU6050Manager>();
    ret = mpu6050Manager->init(runtimeConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050Manager");
        return ret;
    }

    anglePidController = std::make_shared<PIDController>(PIDControllerType::ANGLE);
    ret = anglePidController->init(runtimeConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Angle PIDController");
        return ret;
    }

    speedPidController = std::make_shared<PIDController>(PIDControllerType::SPEED);
    ret = speedPidController->init(runtimeConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Speed PIDController");
        return ret;
    }

    registerObserver(anglePidController);
    registerObserver(speedPidController);

    ESP_LOGI(TAG, "ComponentHandler initialization complete");
    return ESP_OK;
}

void ComponentHandler::registerObserver(std::shared_ptr<IConfigObserver> observer) {
    observers.push_back(observer);
    ESP_LOGD(TAG, "Registered observer");
}

void ComponentHandler::unregisterObserver(std::shared_ptr<IConfigObserver> observer) {
    observers.erase(std::remove(observers.begin(), observers.end(), observer), observers.end());
    ESP_LOGD(TAG, "Unregistered observer");
}

void ComponentHandler::notifyConfigUpdate() {
    ESP_LOGI(TAG, "Notifying all observers of configuration update");
    for (auto observer : observers) {
        esp_err_t ret = observer->onConfigUpdate(runtimeConfig);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Observer failed to update configuration");
        }
    }
}