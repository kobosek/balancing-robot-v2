// main/include/MotorService.hpp
#pragma once

#include "EventBus.hpp"                 // Found via INCLUDE_DIRS
#include "MX1616H_HWDriver.hpp"         // Found via INCLUDE_DIRS
#include "EventHandler.hpp"             // For EventHandler base class
#include "config/MotorConfig.hpp"
// #include "esp_log.h" // Moved to .cpp
#include <memory>
#include <mutex>
#include <algorithm>                    // Not needed in header
#include <cmath>                        // Not needed in header
 // Forward declare event class
class BaseEvent; // <<< Already defined
class MOTOR_OutputEnabledChanged;

class MotorService : public EventHandler {
public:
    MotorService(const MotorConfig& config, EventBus& bus);
    ~MotorService() = default;

    // Declarations only
    esp_err_t init();
    esp_err_t setMotorEffort(float leftEffort, float rightEffort, uint64_t armId = 0, uint32_t generation = 0,
                             int64_t sampleTimestampUs = 0, int64_t maxAgeUs = 0);
    void inhibitImu(uint64_t armId);
    bool isArmAllowed(uint64_t armId, uint32_t generation);

    // Event handling via EventHandler interface
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }
    
    // Kept for backward compatibility
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "MotorService";
    const MotorConfig m_config;
    EventBus& m_eventBus;

    // std::unique_ptr<IMotorHWDriver> m_hw_driver_left; // If using interface
    // std::unique_ptr<IMotorHWDriver> m_hw_driver_right;
    std::unique_ptr<MX1616H_HWDriver> m_hw_driver_left;
    std::unique_ptr<MX1616H_HWDriver> m_hw_driver_right;

    // Serializes the enable decision and both motor writes with the stop path.
    std::mutex m_outputMutex;
    uint64_t m_armId = 0, m_revokedArm = 0;
    uint32_t m_generation = 0;
    bool m_enabled = false; // Protected by m_outputMutex after initialization.
    uint32_t m_pwm_max_duty = 0;

    // Event handlers for specific event types
    void handleMotorOutputEnabledChanged(const MOTOR_OutputEnabledChanged& event);
    // Declaration only
    esp_err_t configureLEDCTimer();
    // Caller holds m_outputMutex; no event publication or sensor operations.
    esp_err_t writeDutyLocked(uint32_t leftDuty1, uint32_t leftDuty2,
                              uint32_t rightDuty1, uint32_t rightDuty2);
};
