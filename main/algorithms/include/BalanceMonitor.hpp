#pragma once

#include "EventBus.hpp"
#include "EventHandler.hpp"
#include "config/SystemBehaviorConfig.hpp"
#include "esp_log.h"
#include "esp_timer.h"

class BaseEvent;
class BALANCE_MonitorModeChanged;
class CONFIG_FullConfigUpdate;
class IMU_OrientationData;

class BalanceMonitor : public EventHandler {
public:
    BalanceMonitor(EventBus& bus, const SystemBehaviorConfig& config);
    ~BalanceMonitor() = default;

    void reset();

    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return "BalanceMonitor"; }

private:
    static constexpr const char* TAG = "BalanceMonitor";
    EventBus& m_eventBus;

    // Fall detection state
    float m_pitch_threshold_rad;
    uint64_t m_threshold_duration_us;
    bool m_potentially_fallen = false;
    int64_t m_fall_start_time_us = 0;
    bool m_fall_event_published = false;

    // Auto balancing detection state
    float m_auto_balance_angle_threshold_rad;
    uint64_t m_auto_balance_hold_time_us;
    bool m_within_auto_balance_angle = false;
    int64_t m_auto_balance_angle_start_time_us = 0;

    bool m_autoBalancingActive = false;
    bool m_fallDetectionActive = false;

    void applyConfig(const SystemBehaviorConfig& config);
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);
    void handleOrientationData(const IMU_OrientationData& event);
    void handleMonitorModeChanged(const BALANCE_MonitorModeChanged& event);

    void checkFall(float pitch_rad);
    void checkAutoBalancing(float pitch_rad);
};
