// main/MotorService.cpp
#include "MotorService.hpp"             // Relative path within module's include dir
#include "MOTOR_OutputEnabledChanged.hpp"
#include "CONFIG_MotorConfigUpdate.hpp"
#include "MX1616H_HWDriver.hpp"         // Found via INCLUDE_DIRS (needed for make_unique)
#include "BaseEvent.hpp"                // Found via INCLUDE_DIRS (needed for handle state change)
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "control_math/EffortMapping.hpp"
#include <cmath>
#include <algorithm>
#include "esp_log.h"                    // Moved from header
#include <memory>                        // Moved from header


MotorService::MotorService(const MotorConfig& config, EventBus& bus,
                           int64_t watchdogTimeoutUs) :
    m_config(config),
    m_eventBus(bus),
    m_watchdogTimeoutUs(std::max<int64_t>(1000, watchdogTimeoutUs)),
    m_enabled(false),
    m_pwm_max_duty(0)
{
    calculateMaxDutyLocked();
    ESP_LOGI(TAG, "PWM Max Duty calculated: %lu for %d bits resolution", m_pwm_max_duty, m_config.duty_resolution);
    createDriversLocked();
}

MotorService::~MotorService()
{
    {
        std::lock_guard<std::mutex> lock(m_outputMutex);
        m_enabled = false;
        stopWatchdogLocked();
        (void)writeDutyLocked(0, 0, 0, 0);
    }
    if (m_watchdogTimer) {
        (void)esp_timer_delete(m_watchdogTimer);
        m_watchdogTimer = nullptr;
    }
}

void MotorService::calculateMaxDutyLocked()
{
    if (m_config.duty_resolution >= 1 && m_config.duty_resolution <= 20) {
        m_pwm_max_duty = (1u << m_config.duty_resolution) - 1u;
        m_configurationValid = true;
    } else {
        m_pwm_max_duty = 0;
        m_configurationValid = false;
        ESP_LOGE(TAG, "Invalid PWM duty resolution: %d", m_config.duty_resolution);
    }
}

void MotorService::createDriversLocked()
{
    m_hw_driver_left = std::make_unique<MX1616H_HWDriver>(
        static_cast<gpio_num_t>(m_config.left_pin_in1),
        static_cast<gpio_num_t>(m_config.left_pin_in2),
        static_cast<ledc_channel_t>(m_config.left_channel_1),
        static_cast<ledc_channel_t>(m_config.left_channel_2),
        static_cast<ledc_timer_t>(m_config.timer_num),
        static_cast<ledc_mode_t>(m_config.speed_mode),
        static_cast<ledc_timer_bit_t>(m_config.duty_resolution),
        m_config.pwm_frequency_hz
    );
    m_hw_driver_right = std::make_unique<MX1616H_HWDriver>(
        static_cast<gpio_num_t>(m_config.right_pin_in1),
        static_cast<gpio_num_t>(m_config.right_pin_in2),
        static_cast<ledc_channel_t>(m_config.right_channel_1),
        static_cast<ledc_channel_t>(m_config.right_channel_2),
        static_cast<ledc_timer_t>(m_config.timer_num),
        static_cast<ledc_mode_t>(m_config.speed_mode),
        static_cast<ledc_timer_bit_t>(m_config.duty_resolution),
        m_config.pwm_frequency_hz
    );
}

esp_err_t MotorService::rebuildDriversLocked()
{
    createDriversLocked();
    if (!m_hw_driver_left || !m_hw_driver_right || !m_configurationValid) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = configureLEDCTimer();
    if (ret != ESP_OK) return ret;
    ret = m_hw_driver_left->init();
    if (ret != ESP_OK) return ret;
    ret = m_hw_driver_right->init();
    return ret;
}

esp_err_t MotorService::init() {
    ESP_LOGI(TAG, "Initializing MotorService...");

    esp_timer_create_args_t watchdogArgs = {};
    watchdogArgs.callback = &MotorService::watchdogCallback;
    watchdogArgs.arg = this;
    watchdogArgs.dispatch_method = ESP_TIMER_TASK;
    watchdogArgs.name = "motor_commit_wd";
    esp_err_t ret = esp_timer_create(&watchdogArgs, &m_watchdogTimer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create motor output watchdog: %s", esp_err_to_name(ret));
        m_watchdogTimer = nullptr;
        return ret;
    }

    ret = configureLEDCTimer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Aborting MotorService init due to timer config failure.");
        (void)esp_timer_delete(m_watchdogTimer);
        m_watchdogTimer = nullptr;
        return ret;
    }
    ESP_LOGI(TAG, "LEDC Timer %d configuration verified/completed.", (int)m_config.timer_num);

    if (m_hw_driver_left == nullptr) {
        (void)esp_timer_delete(m_watchdogTimer);
        m_watchdogTimer = nullptr;
        return ESP_ERR_INVALID_STATE;
    }
    ret = m_hw_driver_left->init();
    if (ret != ESP_OK) {
        (void)esp_timer_delete(m_watchdogTimer);
        m_watchdogTimer = nullptr;
        ESP_LOGE(TAG, "Failed to initialize Left HW Driver: %s", esp_err_to_name(ret));
        return ret;
    }

    if (m_hw_driver_right == nullptr) {
        (void)esp_timer_delete(m_watchdogTimer);
        m_watchdogTimer = nullptr;
        return ESP_ERR_INVALID_STATE;
    }
    ret = m_hw_driver_right->init();
    if (ret != ESP_OK) {
        (void)esp_timer_delete(m_watchdogTimer);
        m_watchdogTimer = nullptr;
        ESP_LOGE(TAG, "Failed to initialize Right HW Driver: %s", esp_err_to_name(ret));
        return ret;
    }

    m_enabled = false;
    ret = setMotorEffort(0.0f, 0.0f);
    if(ret != ESP_OK) { ESP_LOGE(TAG, "Failed to set initial motor effort to zero during init!"); }

    ESP_LOGI(TAG, "MotorService Initialized Successfully.");
    return ESP_OK;
}

// EventHandler implementation
void MotorService::handleEvent(const BaseEvent& event) {
    if (event.is<MOTOR_OutputEnabledChanged>()) {
        handleMotorOutputEnabledChanged(event.as<MOTOR_OutputEnabledChanged>());
    } else if (event.is<CONFIG_MotorConfigUpdate>()) {
        handleMotorConfigUpdate(event.as<CONFIG_MotorConfigUpdate>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

void MotorService::watchdogCallback(void* arg)
{
    auto* service = static_cast<MotorService*>(arg);
    if (service) service->handleWatchdog();
}

void MotorService::stopWatchdogLocked()
{
    if (m_watchdogTimer) {
        // A one-shot timer may already have fired; both STOPPED and
        // INVALID_STATE are harmless here because the output is being
        // inhibited synchronously under the same mutex.
        (void)esp_timer_stop(m_watchdogTimer);
    }
    m_lastAcceptedCommitUs = 0;
}

void MotorService::armWatchdog()
{
    if (!m_watchdogTimer || m_watchdogTimeoutUs <= 0) return;
    (void)esp_timer_stop(m_watchdogTimer);
    const esp_err_t ret = esp_timer_start_once(
        m_watchdogTimer, static_cast<uint64_t>(m_watchdogTimeoutUs));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to arm motor output watchdog: %s", esp_err_to_name(ret));
        std::lock_guard<std::mutex> lock(m_outputMutex);
        m_enabled = false;
        m_revokedArm = std::max(m_revokedArm, m_armId);
        (void)writeDutyLocked(0, 0, 0, 0);
    }
}

void MotorService::handleWatchdog()
{
    esp_err_t stopResult = ESP_OK;
    bool expired = false;
    {
        std::lock_guard<std::mutex> lock(m_outputMutex);
        if (!m_enabled || m_lastAcceptedCommitUs <= 0) return;
        const int64_t now = esp_timer_get_time();
        expired = now < m_lastAcceptedCommitUs ||
            now - m_lastAcceptedCommitUs >= m_watchdogTimeoutUs;
        if (!expired) return;
        m_enabled = false;
        m_revokedArm = std::max(m_revokedArm, m_armId);
        stopResult = writeDutyLocked(0, 0, 0, 0);
        m_lastAcceptedCommitUs = 0;
    }
    ESP_LOGW(TAG, "Motor output watchdog expired; arm revoked and PWM disabled");
    if (stopResult != ESP_OK) {
        ESP_LOGE(TAG, "Watchdog stop could not update all motor channels: %s",
                 esp_err_to_name(stopResult));
    }
}

void MotorService::handleMotorConfigUpdate(const CONFIG_MotorConfigUpdate& event)
{
    std::lock_guard<std::mutex> lock(m_outputMutex);
    const MotorConfig previous = m_config;
    if (previous == event.config) return;

    // ConfigurationService serializes this event while the CONFIGURATION
    // operation lease is held and StateManager keeps motion disabled.  Still
    // force a synchronous zero before replacing either driver so a stale
    // output cannot survive a pin/channel change.
    m_enabled = false;
    stopWatchdogLocked();
    const esp_err_t stopResult = writeDutyLocked(0, 0, 0, 0);
    if (stopResult != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop motors before configuration apply: %s",
                 esp_err_to_name(stopResult));
    }

    const bool hardwareRestart = event.requiresHardwareInit;
    if (hardwareRestart) {
        m_configurationValid = false;
        m_hw_driver_left.reset();
        m_hw_driver_right.reset();
    }
    m_config = event.config;
    calculateMaxDutyLocked();

    esp_err_t ret = ESP_OK;
    if (hardwareRestart) {
        ret = rebuildDriversLocked();
    } else {
        // Deadzone and other actuator-boundary fields can be applied without
        // touching LEDC.  The next enable event will use this complete
        // snapshot and the watchdog remains disarmed until then.
        ret = (m_pwm_max_duty != 0 && m_hw_driver_left && m_hw_driver_right)
            ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    if (ret != ESP_OK) {
        m_configurationValid = false;
        ESP_LOGE(TAG, "Motor configuration was not applied: %s",
                 esp_err_to_name(ret));
        return;
    }
    m_configurationValid = true;
    ESP_LOGI(TAG, "Motor configuration applied (%s)",
             event.requiresHardwareInit ? "hardware restarted" : "actuator fields updated");
}

esp_err_t MotorService::configureLEDCTimer() {
     ESP_LOGI(TAG, "Configuring LEDC Timer %d: Freq=%luHz, Res=%d bits, Mode=%d",
              (int)m_config.timer_num, m_config.pwm_frequency_hz, m_config.duty_resolution, (int)m_config.speed_mode);

     if (m_config.duty_resolution < 1 || m_config.duty_resolution > 20) {
          ESP_LOGE(TAG, "Configuration error: Invalid duty resolution (%d).",
                   m_config.duty_resolution);
          return ESP_ERR_INVALID_ARG;
     }

     if (m_config.speed_mode != LEDC_LOW_SPEED_MODE
#if SOC_LEDC_SUPPORT_HS_MODE
         && m_config.speed_mode != LEDC_HIGH_SPEED_MODE
#endif
     ) {
          ESP_LOGE(TAG, "Configuration error: Invalid speed mode (%d) requested in config.", (int)m_config.speed_mode);
          return ESP_ERR_INVALID_ARG;
     }

     ledc_timer_config_t ledc_timer = {
        .speed_mode       = static_cast<ledc_mode_t>(m_config.speed_mode),
        .duty_resolution  = static_cast<ledc_timer_bit_t>(m_config.duty_resolution),
        .timer_num        = static_cast<ledc_timer_t>(m_config.timer_num),
        .freq_hz          = m_config.pwm_frequency_hz,
        .clk_cfg          = LEDC_AUTO_CLK};

    esp_err_t ret = ledc_timer_config(&ledc_timer);

    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "LEDC timer %d already configured. Assuming compatible settings.", (int)m_config.timer_num);
        return ESP_OK;
    } else if (ret != ESP_OK) {
         ESP_LOGE(TAG, "ledc_timer_config failed for timer %d: %s (%d)", (int)m_config.timer_num, esp_err_to_name(ret), ret);
         return ret;
    }

    ESP_LOGI(TAG, "LEDC timer %d configured successfully for speed mode %d.", (int)m_config.timer_num, (int)m_config.speed_mode);
    return ESP_OK;
}

esp_err_t MotorService::setMotorEffort(float leftEffort, float rightEffort, uint64_t armId, uint32_t generation,
                                        int64_t sampleTimestampUs, int64_t maxAgeUs) {
    const bool finite = std::isfinite(leftEffort) && std::isfinite(rightEffort);
    if (!finite) {
        leftEffort = rightEffort = 0.0f;
    }
    leftEffort = std::max(-1.0f, std::min(1.0f, leftEffort));
    rightEffort = std::max(-1.0f, std::min(1.0f, rightEffort));
    uint32_t leftDuty1 = 0, leftDuty2 = 0, rightDuty1 = 0, rightDuty2 = 0;
    esp_err_t result = ESP_OK;
    bool expired = false;
    bool accepted = false;
    bool configurationInvalid = false;
    {
        std::lock_guard<std::mutex> lock(m_outputMutex);
        const auto now = esp_timer_get_time();
        expired = maxAgeUs > 0 && (sampleTimestampUs <= 0 || now < sampleTimestampUs || now - sampleTimestampUs > maxAgeUs);
        if (expired) { m_revokedArm = std::max(m_revokedArm, armId); if (m_armId <= m_revokedArm) m_enabled = false; }
        if (!finite) { m_revokedArm = std::max(m_revokedArm, m_armId); m_enabled = false; }
        configurationInvalid = !m_configurationValid || m_pwm_max_duty == 0 ||
            !m_hw_driver_left || !m_hw_driver_right;
        if (!configurationInvalid) {
            const uint32_t leftDuty = control_math::effortToPwmDuty(
                leftEffort, m_pwm_max_duty, m_config.deadzone_duty);
            const uint32_t rightDuty = control_math::effortToPwmDuty(
                rightEffort, m_pwm_max_duty, m_config.deadzone_duty);
            if (leftDuty != 0) {
                if (leftEffort > 0.0f) { leftDuty1 = leftDuty; }
                else { leftDuty2 = leftDuty; }
            }
            if (rightDuty != 0) {
                if (rightEffort > 0.0f) { rightDuty1 = rightDuty; }
                else { rightDuty2 = rightDuty; }
            }
        }
        accepted = finite && !expired && !configurationInvalid && m_enabled &&
            armId == m_armId && generation == m_generation && armId > m_revokedArm;
        if (!accepted) {
            leftDuty1 = leftDuty2 = rightDuty1 = rightDuty2 = 0;
        }
        result = writeDutyLocked(leftDuty1, leftDuty2, rightDuty1, rightDuty2);
        if (accepted && result == ESP_OK) {
            m_lastAcceptedCommitUs = now;
        } else if (accepted && result != ESP_OK) {
            // A partial output write is unsafe. Revoke the arm and make a
            // second, full zero-duty attempt before returning the error.
            m_enabled = false;
            m_revokedArm = std::max(m_revokedArm, m_armId);
            stopWatchdogLocked();
            const esp_err_t zeroResult = writeDutyLocked(0, 0, 0, 0);
            if (result == ESP_OK) result = zeroResult;
        }
    }
    if (accepted && result == ESP_OK) armWatchdog();
    if (configurationInvalid) return ESP_ERR_INVALID_STATE;
    return result != ESP_OK ? result : (expired ? ESP_ERR_TIMEOUT : (finite ? ESP_OK : ESP_ERR_INVALID_ARG));
}

esp_err_t MotorService::writeDutyLocked(uint32_t leftDuty1, uint32_t leftDuty2,
                                        uint32_t rightDuty1, uint32_t rightDuty2) {
    // Always attempt both sides, including when one driver reports an error.
    const esp_err_t leftResult = m_hw_driver_left
        ? m_hw_driver_left->setRawDuty(leftDuty1, leftDuty2) : ESP_ERR_INVALID_STATE;
    const esp_err_t rightResult = m_hw_driver_right
        ? m_hw_driver_right->setRawDuty(rightDuty1, rightDuty2) : ESP_ERR_INVALID_STATE;
    return leftResult != ESP_OK ? leftResult : rightResult;
}

void MotorService::handleMotorOutputEnabledChanged(const MOTOR_OutputEnabledChanged& event) {
    esp_err_t stopResult = ESP_OK;
    bool arm = false;
    {
        std::lock_guard<std::mutex> lock(m_outputMutex);
        if (event.armId < m_armId) return;
        m_armId = event.armId;
        m_generation = event.generation;
        if (!event.enabled || !m_configurationValid) m_revokedArm = std::max(m_revokedArm, event.armId);
        m_enabled = event.enabled && m_configurationValid && event.armId > m_revokedArm;
        if (m_enabled) {
            m_lastAcceptedCommitUs = esp_timer_get_time();
            arm = true;
        } else {
            stopWatchdogLocked();
        }
        if (!m_enabled) {
            // Repeat the zero write even for an already-disabled state, so a
            // previous failed stop can be retried. Do not recursively lock.
            stopResult = writeDutyLocked(0, 0, 0, 0);
        }
    }
    if (stopResult != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop motors: %s", esp_err_to_name(stopResult));
    }
    if (arm) armWatchdog();
}

void MotorService::inhibitImu(uint64_t armId) {
    std::lock_guard<std::mutex> lock(m_outputMutex);
    m_revokedArm = std::max(m_revokedArm, armId);
    if (m_armId <= m_revokedArm) {
        m_enabled = false;
        stopWatchdogLocked();
        (void)writeDutyLocked(0, 0, 0, 0);
    }
}
bool MotorService::isArmAllowed(uint64_t armId, uint32_t generation) {
    std::lock_guard<std::mutex> lock(m_outputMutex);
    return m_configurationValid && m_enabled && armId == m_armId &&
        generation == m_generation && armId > m_revokedArm;
}
