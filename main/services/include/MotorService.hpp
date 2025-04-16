// main/include/MotorService.hpp
#pragma once

#include "ConfigData.hpp"               // Found via INCLUDE_DIRS
#include "EventBus.hpp"                 // Found via INCLUDE_DIRS
#include "SystemState.hpp"              // Found via INCLUDE_DIRS
#include "MX1616H_HWDriver.hpp"         // Found via INCLUDE_DIRS
// #include "esp_log.h" // Moved to .cpp
#include <memory>
#include <algorithm>                    // Not needed in header
#include <cmath>                        // Not needed in header
 // Forward declare event class
class BaseEvent; // <<< Already defined

class MotorService {
public:
    MotorService(const MotorConfig& config, EventBus& bus);
    ~MotorService() = default;

    // Declarations only
    esp_err_t init();
    esp_err_t setMotorEffort(float leftEffort, float rightEffort);

    // <<< ADDED: Method for event subscriptions >>>
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "MotorService";
    const MotorConfig m_config;
    EventBus& m_eventBus;

    // std::unique_ptr<IMotorHWDriver> m_hw_driver_left; // If using interface
    // std::unique_ptr<IMotorHWDriver> m_hw_driver_right;
    std::unique_ptr<MX1616H_HWDriver> m_hw_driver_left;
    std::unique_ptr<MX1616H_HWDriver> m_hw_driver_right;

    SystemState m_current_system_state = SystemState::INIT;
    bool m_enabled = false;
    uint32_t m_pwm_max_duty = 0;

    // Declaration only
    void handleSystemStateChange(const BaseEvent& event);
    // Declaration only
    esp_err_t configureLEDCTimer();
};