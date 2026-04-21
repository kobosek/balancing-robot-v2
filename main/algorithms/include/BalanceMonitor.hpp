#pragma once

#include "EventBus.hpp"
#include "EventHandler.hpp"
#include "ConfigData.hpp"
#include "SystemState.hpp"
#include "esp_log.h"
#include "esp_timer.h"

class BaseEvent;
class CONFIG_FullConfigUpdate;
class IMU_OrientationData;
class SYSTEM_StateChanged;
class UI_EnableFallRecovery;
class UI_DisableFallRecovery;
class UI_DisableFallDetection;
class UI_EnableFallDetection;

class BalanceMonitor : public EventHandler {
public:
    BalanceMonitor(EventBus& bus, const SystemBehaviorConfig& config);
    ~BalanceMonitor() = default;

    void reset();

    bool isAutoRecoveryEnabled() const { return m_autoRecoveryEnabled; }
    bool isFallDetectionEnabled() const { return m_fallDetectionEnabled; }

    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return "BalanceMonitor"; }

private:
    static constexpr const char* TAG = "BalanceMonitor";
    EventBus& m_eventBus;
    SystemState m_currentState = SystemState::INIT;

    // Fall detection state
    float m_pitch_threshold_rad;
    uint64_t m_threshold_duration_us;
    bool m_potentially_fallen = false;
    int64_t m_fall_start_time_us = 0;
    bool m_fall_event_published = false;

    // Recovery detection state
    float m_recovery_angle_threshold_rad;
    uint64_t m_recovery_hold_time_us;
    bool m_within_recovery_angle = false;
    int64_t m_recovery_angle_start_time_us = 0;

    bool m_autoRecoveryEnabled = true;
    bool m_fallDetectionEnabled = true;

    void applyConfig(const SystemBehaviorConfig& config);
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);
    void handleOrientationData(const IMU_OrientationData& event);
    void handleStateChanged(const SYSTEM_StateChanged& event);
    void handleEnableRecovery(const UI_EnableFallRecovery& event);
    void handleDisableRecovery(const UI_DisableFallRecovery& event);
    void handleEnableFallDetect(const UI_EnableFallDetection& event);
    void handleDisableFallDetect(const UI_DisableFallDetection& event);

    void checkFall(float pitch_rad);
    void checkRecovery(float pitch_rad);
};
