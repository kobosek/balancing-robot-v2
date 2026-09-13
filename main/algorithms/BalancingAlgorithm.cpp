// ================================================
// File: main/algorithms/BalancingAlgorithm.cpp
// ================================================
#include "BalancingAlgorithm.hpp"
#include "LongitudinalCascadeBalanceStrategy.hpp"
#include "NestedPidBalanceStrategy.hpp"
#include "CONFIG_FullConfigUpdate.hpp"
#include "CONFIG_PidConfigUpdate.hpp"
#include "BaseEvent.hpp"
#include "esp_check.h"
#include <cmath>

namespace {
ConfigData makeInitialConfig(const PIDConfig& initialAnglePid,
                             const PIDConfig& initialSpeedLeftPid,
                             const PIDConfig& initialSpeedRightPid,
                             const PIDConfig& initialYawAnglePid,
                             const PIDConfig& initialYawRatePid,
                             const ControlConfig& initialControl,
                             const EncoderConfig& initialEncoder,
                             const RobotDimensionsConfig& initialDimensions)
{
    ConfigData config;
    config.control = initialControl;
    config.control.strategies.nested_pid.angle = initialAnglePid;
    config.control.strategies.nested_pid.speed_left = initialSpeedLeftPid;
    config.control.strategies.nested_pid.speed_right = initialSpeedRightPid;
    config.control.strategies.nested_pid.yaw_angle = initialYawAnglePid;
    config.control.strategies.nested_pid.yaw_rate = initialYawRatePid;
    config.control.strategies.nested_pid.max_target_pitch_offset_deg =
        initialControl.max_target_pitch_offset_deg;
    config.control.strategies.nested_pid.yaw_control_enabled =
        initialControl.yaw_control_enabled;
    config.encoder = initialEncoder;
    config.dimensions = initialDimensions;
    return config;
}
}

BalancingAlgorithm::BalancingAlgorithm(EventBus& eventBus,
                                       const ConfigData& initialConfig) :
    m_eventBus(eventBus),
    m_strategy(std::make_unique<NestedPidBalanceStrategy>())
{
    ESP_LOGI(TAG, "Balancing Algorithm created.");
    applyConfig(initialConfig);
}

// Compatibility constructor for existing isolated callers. New composition
// code should pass the complete ConfigData snapshot above.
BalancingAlgorithm::BalancingAlgorithm(EventBus& eventBus,
                                       const PIDConfig& initialAnglePid,
                                       const PIDConfig& initialSpeedLeftPid,
                                       const PIDConfig& initialSpeedRightPid,
                                       const PIDConfig& initialYawAnglePid,
                                       const PIDConfig& initialYawRatePid,
                                       const ControlConfig& initialControl,
                                       const EncoderConfig& initialEncoder,
                                       const RobotDimensionsConfig& initialDimensions) :
    BalancingAlgorithm(eventBus,
        makeInitialConfig(initialAnglePid,
                          initialSpeedLeftPid,
                          initialSpeedRightPid,
                          initialYawAnglePid,
                          initialYawRatePid,
                          initialControl,
                          initialEncoder,
                          initialDimensions)) {}

esp_err_t BalancingAlgorithm::init() {
    ESP_LOGI(TAG, "Initializing Balancing Algorithm...");
    // Active strategy is already initialized in constructor with initial config.
    resetState();
    ESP_LOGI(TAG, "Balancing Algorithm Initialized.");
    return ESP_OK;
}

void BalancingAlgorithm::handleEvent(const BaseEvent& event) {
    if (event.is<CONFIG_FullConfigUpdate>()) {
        handleConfigUpdate(event.as<CONFIG_FullConfigUpdate>());
    } else if (event.is<CONFIG_PidConfigUpdate>()) {
        handlePIDConfigUpdate(event.as<CONFIG_PidConfigUpdate>());
    } else if (event.is<CONTROL_RunModeChanged>()) {
        handleRunModeChanged(event.as<CONTROL_RunModeChanged>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

void BalancingAlgorithm::resetState() {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    if (m_strategy) {
        m_strategy->reset();
    }
}

MotorEffort BalancingAlgorithm::update(float dt, float currentPitch_deg, float currentPitchRate_dps,
                                      float currentYaw_deg,
                                      float currentYawRate_dps,
                                      float currentSpeedLeft_dps, float currentSpeedRight_dps,
                                      float targetPitchOffset_deg, float targetAngVel_dps,
                                      const LongitudinalOdometryResult& odometry,
                                      int64_t nowUs,
                                      int64_t motionTimeoutUs,
                                      const LongitudinalMotionCommand& motion,
                                      uint64_t controlArmId)
{
    return updateDetailed(dt, currentPitch_deg, currentPitchRate_dps,
                          currentYaw_deg, currentYawRate_dps,
                          currentSpeedLeft_dps, currentSpeedRight_dps,
                          targetPitchOffset_deg, targetAngVel_dps, odometry,
                          nowUs, motionTimeoutUs, motion, controlArmId).effort;
}

BalanceControlResult BalancingAlgorithm::updateDetailed(
    float dt, float currentPitch_deg, float currentPitchRate_dps,
    float currentYaw_deg, float currentYawRate_dps,
    float currentSpeedLeft_dps, float currentSpeedRight_dps,
    float targetPitchOffset_deg, float targetAngVel_dps,
    const LongitudinalOdometryResult& odometry,
    int64_t nowUs,
    int64_t motionTimeoutUs,
    const LongitudinalMotionCommand& motion,
    uint64_t controlArmId)
{
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    BalanceControlResult result = {};
    result.strategyId = m_activeStrategyId;
    result.configurationRevision = m_activeStrategyRevision;
    result.configRevision = m_appliedConfigRevision;
    if (!m_strategy) {
        return result;
    }

    const BalanceControlInput input = {
        dt,
        currentPitch_deg,
        currentPitchRate_dps,
        currentYaw_deg,
        currentYawRate_dps,
        currentSpeedLeft_dps,
        currentSpeedRight_dps,
        targetPitchOffset_deg,
        targetAngVel_dps,
        nowUs,
        motionTimeoutUs,
        motion,
        odometry,
        controlArmId
    };
    result.effort = m_strategy->update(input);
    result.strategyId = m_activeStrategyId;
    result.configurationRevision = m_activeStrategyRevision;
    result.configRevision = m_appliedConfigRevision;
    result.diagnostics = m_strategy->getDiagnostics();
    result.speedSetpointLeft_dps = m_strategy->getLastSpeedSetpointLeftDPS();
    result.speedSetpointRight_dps = m_strategy->getLastSpeedSetpointRightDPS();
    result.targetYaw_deg = m_strategy->getLastTargetYawDeg();
    result.desiredYawRate_dps = m_strategy->getLastDesiredYawRateDPS();
    result.valid = result.diagnostics.valid &&
        std::isfinite(result.effort.left) && std::isfinite(result.effort.right);
    return result;
}

// Helper to apply config values
void BalancingAlgorithm::applyConfig(const ConfigData& config) {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    if (m_hasConfigRevision && config.config_revision < m_appliedConfigRevision) {
        ESP_LOGW(TAG, "Ignoring stale full config revision %lu (current %lu)",
                 static_cast<unsigned long>(config.config_revision),
                 static_cast<unsigned long>(m_appliedConfigRevision));
        return;
    }
    const BalanceStrategyId requestedStrategy = config.control.strategies.active;
    const uint32_t requestedStrategyRevision = requestedStrategy == BalanceStrategyId::NESTED_PID
        ? config.control.strategies.nested_pid.revision
        : config.control.strategies.longitudinal_cascade.revision;
    if (m_hasConfigRevision && requestedStrategy == m_activeStrategyId &&
        config.config_revision == m_appliedConfigRevision &&
        requestedStrategyRevision < m_activeStrategyRevision) {
        ESP_LOGW(TAG, "Ignoring stale strategy revision %lu (current %lu)",
                 static_cast<unsigned long>(requestedStrategyRevision),
                 static_cast<unsigned long>(m_activeStrategyRevision));
        return;
    }
    if (requestedStrategy != m_activeStrategyId) {
        if (m_controlMode != ControlRunMode::DISABLED) {
            ESP_LOGW(TAG, "Ignoring strategy change while control mode is active");
            return;
        }
        auto replacement = createStrategy(requestedStrategy);
        if (!replacement) {
            ESP_LOGW(TAG, "Unsupported balance strategy requested");
            return;
        }
        m_strategy = std::move(replacement);
        m_activeStrategyId = requestedStrategy;
        ESP_LOGI(TAG, "Selected balance strategy '%s'", m_strategy->name());
    }
    if (!m_strategy) {
        return;
    }
    ESP_LOGD(TAG, "Applying new config to balance strategy '%s'.", m_strategy->name());
    m_strategy->applyConfig(config);
    m_activeStrategyRevision = m_activeStrategyId == BalanceStrategyId::NESTED_PID
        ? config.control.strategies.nested_pid.revision
        : config.control.strategies.longitudinal_cascade.revision;
    m_appliedConfigRevision = config.config_revision;
    m_hasConfigRevision = true;
}

std::unique_ptr<IBalanceControlStrategy> BalancingAlgorithm::createStrategy(BalanceStrategyId id) const {
    switch (id) {
        case BalanceStrategyId::NESTED_PID:
            return std::make_unique<NestedPidBalanceStrategy>();
        case BalanceStrategyId::LONGITUDINAL_CASCADE:
            return std::make_unique<LongitudinalCascadeBalanceStrategy>();
        default:
            return nullptr;
    }
}

// Handle config update event
void BalancingAlgorithm::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
     ESP_LOGD(TAG, "Handling general config update event."); // Use DEBUG level
     applyConfig(event.configData); // Apply the full config payload
}

// Handle granular PID config update event
void BalancingAlgorithm::handlePIDConfigUpdate(const CONFIG_PidConfigUpdate& event) {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    if (!m_strategy || event.strategyId != m_activeStrategyId) {
        return;
    }
    if (m_hasConfigRevision &&
        (event.configRevision < m_appliedConfigRevision ||
         (event.configRevision == 0 && m_appliedConfigRevision > 0) ||
         event.strategyRevision < m_activeStrategyRevision)) {
        ESP_LOGW(TAG, "Ignoring stale granular PID update '%s'", event.pidName.c_str());
        return;
    }
    m_strategy->updatePidConfig(event.pidName, event.config);
    if (!m_hasConfigRevision || event.configRevision > m_appliedConfigRevision) {
        m_appliedConfigRevision = event.configRevision;
    }
    if (event.strategyRevision > m_activeStrategyRevision) {
        m_activeStrategyRevision = event.strategyRevision;
    }
}

void BalancingAlgorithm::handleRunModeChanged(const CONTROL_RunModeChanged& event) {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    if (event.armId < m_controlArmId) {
        return;
    }
    m_controlArmId = event.armId;
    m_controlMode = event.mode;
}

float BalancingAlgorithm::getLastSpeedSetpointLeftDPS() const {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    return m_strategy ? m_strategy->getLastSpeedSetpointLeftDPS() : 0.0f;
}

float BalancingAlgorithm::getLastSpeedSetpointRightDPS() const {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    return m_strategy ? m_strategy->getLastSpeedSetpointRightDPS() : 0.0f;
}

float BalancingAlgorithm::getLastTargetYawDeg() const {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    return m_strategy ? m_strategy->getLastTargetYawDeg() : 0.0f;
}

float BalancingAlgorithm::getLastDesiredYawRateDPS() const {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    return m_strategy ? m_strategy->getLastDesiredYawRateDPS() : 0.0f;
}

bool BalancingAlgorithm::isYawControlEnabled() const {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    return m_strategy ? m_strategy->isYawControlEnabled() : false;
}

BalanceControlDiagnostics BalancingAlgorithm::getDiagnostics() const {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    return m_strategy ? m_strategy->getDiagnostics() : BalanceControlDiagnostics{};
}

BalanceStrategyId BalancingAlgorithm::getActiveStrategyId() const {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    return m_activeStrategyId;
}

uint32_t BalancingAlgorithm::getActiveStrategyRevision() const {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    return m_activeStrategyRevision;
}

uint32_t BalancingAlgorithm::getAppliedConfigRevision() const {
    std::lock_guard<std::mutex> lock(m_strategyMutex);
    return m_appliedConfigRevision;
}
