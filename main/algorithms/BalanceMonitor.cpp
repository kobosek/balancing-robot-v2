#include "BalanceMonitor.hpp"
#include "BALANCE_FallDetected.hpp"
#include "BALANCE_AutoBalanceReady.hpp"
#include "SYSTEM_StateChanged.hpp"
#include "EventBus.hpp"
#include "CONFIG_FullConfigUpdate.hpp"
#include "IMU_OrientationData.hpp"
#include "UI_DisableFallDetection.hpp"
#include "UI_EnableFallDetection.hpp"
#include "UI_DisableAutoBalancing.hpp"
#include "UI_EnableAutoBalancing.hpp"
#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static constexpr float DEG_TO_RAD = M_PI / 180.0f;
static constexpr float RAD_TO_DEG = 180.0f / M_PI;

BalanceMonitor::BalanceMonitor(EventBus& bus, const SystemBehaviorConfig& config)
    : m_eventBus(bus),
      m_pitch_threshold_rad(0.785f),
      m_threshold_duration_us(500000),
      m_auto_balance_angle_threshold_rad(0.087f),
      m_auto_balance_hold_time_us(2000000),
      m_autoBalancingEnabled(true),
      m_fallDetectionEnabled(true)
{
    applyConfig(config);
    reset();
    ESP_LOGI(TAG, "BalanceMonitor initialized.");
}

void BalanceMonitor::handleEvent(const BaseEvent& event) {
    switch (event.type) {
        case EventType::IMU_ORIENTATION_DATA:
            handleOrientationData(static_cast<const IMU_OrientationData&>(event));
            break;
        case EventType::SYSTEM_STATE_CHANGED:
            handleStateChanged(static_cast<const SYSTEM_StateChanged&>(event));
            break;
        case EventType::CONFIG_FULL_UPDATE:
            handleConfigUpdate(static_cast<const CONFIG_FullConfigUpdate&>(event));
            break;

        case EventType::UI_ENABLE_AUTO_BALANCING:
            handleEnableAutoBalancing(static_cast<const UI_EnableAutoBalancing&>(event));
            break;

        case EventType::UI_DISABLE_AUTO_BALANCING:
            handleDisableAutoBalancing(static_cast<const UI_DisableAutoBalancing&>(event));
            break;

        case EventType::UI_ENABLE_FALL_DETECTION:
            handleEnableFallDetect(static_cast<const UI_EnableFallDetection&>(event));
            break;

        case EventType::UI_DISABLE_FALL_DETECTION:
            handleDisableFallDetect(static_cast<const UI_DisableFallDetection&>(event));
            break;
        default:
            break;
    }
}

void BalanceMonitor::applyConfig(const SystemBehaviorConfig& config) {
    m_pitch_threshold_rad = config.fall_pitch_threshold_deg * DEG_TO_RAD;
    m_threshold_duration_us = config.fall_threshold_duration_ms * 1000ULL;
    m_auto_balance_angle_threshold_rad = config.auto_balance_pitch_threshold_deg * DEG_TO_RAD;
    m_auto_balance_hold_time_us = config.auto_balance_hold_duration_ms * 1000ULL;
    ESP_LOGI(TAG, "Config applied: fall=%.2f deg, fall_dur=%llu us, auto_balance=%.2f deg, auto_balance_dur=%llu us",
             config.fall_pitch_threshold_deg, m_threshold_duration_us,
             config.auto_balance_pitch_threshold_deg, m_auto_balance_hold_time_us);
}

void BalanceMonitor::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
    applyConfig(event.configData.behavior);
}

void BalanceMonitor::reset() {
    m_potentially_fallen = false;
    m_fall_start_time_us = 0;
    m_fall_event_published = false;
    m_within_auto_balance_angle = false;
    m_auto_balance_angle_start_time_us = 0;
    ESP_LOGD(TAG, "State reset.");
}

void BalanceMonitor::handleOrientationData(const IMU_OrientationData& event) {
    if (m_currentState == SystemState::BALANCING && m_fallDetectionEnabled) {
        checkFall(event.pitch_rad);
    }

    if (m_autoBalancingEnabled &&
        (m_currentState == SystemState::IDLE || m_currentState == SystemState::FALLEN)) {
        checkAutoBalancing(event.pitch_rad);
    } else if (m_within_auto_balance_angle) {
        m_within_auto_balance_angle = false;
        m_auto_balance_angle_start_time_us = 0;
    }
}

void BalanceMonitor::handleStateChanged(const SYSTEM_StateChanged& event) {
    m_currentState = event.newState;

    if (event.newState != SystemState::BALANCING) {
        m_potentially_fallen = false;
        m_fall_start_time_us = 0;
        m_fall_event_published = false;
    }

    if (event.newState != SystemState::IDLE && event.newState != SystemState::FALLEN) {
        m_within_auto_balance_angle = false;
        m_auto_balance_angle_start_time_us = 0;
    }
}

void BalanceMonitor::checkFall(float pitch_rad) {
    bool threshold_exceeded = (std::abs(pitch_rad) > m_pitch_threshold_rad);
    int64_t now = esp_timer_get_time();

    if (threshold_exceeded) {
        if (!m_potentially_fallen) {
            m_potentially_fallen = true;
            m_fall_start_time_us = now;
            m_fall_event_published = false;
            ESP_LOGD(TAG, "Potential fall: %.2f deg exceeds threshold %.2f deg",
                     pitch_rad * RAD_TO_DEG, m_pitch_threshold_rad * RAD_TO_DEG);
        } else if (!m_fall_event_published && (now - m_fall_start_time_us) >= (int64_t)m_threshold_duration_us) {
            ESP_LOGW(TAG, "Fall confirmed after %llu us.", m_threshold_duration_us);
            BALANCE_FallDetected fall_event;
            m_eventBus.publish(fall_event);
            m_fall_event_published = true;
        }
    } else {
        if (m_potentially_fallen) {
            ESP_LOGD(TAG, "Potential fall cleared (%.2f deg within threshold).", pitch_rad * RAD_TO_DEG);
            m_potentially_fallen = false;
            m_fall_start_time_us = 0;
            m_fall_event_published = false;
        }
    }
}

void BalanceMonitor::checkAutoBalancing(float pitch_rad) {
    bool is_upright = (std::abs(pitch_rad) < m_auto_balance_angle_threshold_rad);
    int64_t now = esp_timer_get_time();

    if (is_upright) {
        if (!m_within_auto_balance_angle) {
            m_within_auto_balance_angle = true;
            m_auto_balance_angle_start_time_us = now;
            ESP_LOGI(TAG, "Robot upright enough for auto balancing (%.1f deg), starting timer (%llu us)...",
                     pitch_rad * RAD_TO_DEG, m_auto_balance_hold_time_us);
        } else if ((uint64_t)(now - m_auto_balance_angle_start_time_us) >= m_auto_balance_hold_time_us) {
            ESP_LOGI(TAG, "Auto balance conditions met while state=%d.", static_cast<int>(m_currentState));
            m_within_auto_balance_angle = false;
            m_auto_balance_angle_start_time_us = 0;
            BALANCE_AutoBalanceReady auto_balance_event;
            m_eventBus.publish(auto_balance_event);
        }
    } else {
        if (m_within_auto_balance_angle) {
            ESP_LOGI(TAG, "Left auto balance angle window (%.1f deg). Resetting timer.", pitch_rad * RAD_TO_DEG);
            m_within_auto_balance_angle = false;
            m_auto_balance_angle_start_time_us = 0;
        }
    }
}

void BalanceMonitor::handleEnableAutoBalancing(const UI_EnableAutoBalancing& event) {
    (void)event;
    m_autoBalancingEnabled = true;
    m_within_auto_balance_angle = false;
    m_auto_balance_angle_start_time_us = 0;
}

void BalanceMonitor::handleDisableAutoBalancing(const UI_DisableAutoBalancing& event) {
    (void)event;
    m_autoBalancingEnabled = false;
    m_within_auto_balance_angle = false;
    m_auto_balance_angle_start_time_us = 0;
}

void BalanceMonitor::handleEnableFallDetect(const UI_EnableFallDetection& event) {
    (void)event;
    m_fallDetectionEnabled = true;
    m_potentially_fallen = false;
    m_fall_start_time_us = 0;
    m_fall_event_published = false;
}

void BalanceMonitor::handleDisableFallDetect(const UI_DisableFallDetection& event) {
    (void)event;
    m_fallDetectionEnabled = false;
    m_potentially_fallen = false;
    m_fall_start_time_us = 0;
    m_fall_event_published = false;
}
