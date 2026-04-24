#include "BatteryService.hpp"
#include "ConfigData.hpp"
#include "BATTERY_StatusUpdate.hpp"
#include "EventBus.hpp"
#include "CONFIG_BatteryConfigUpdate.hpp"
#include "CONFIG_BehaviorConfigUpdate.hpp"
#include "BaseEvent.hpp"
#include <numeric>
#include <vector>
#include <algorithm>
#include <cmath>
#include "esp_check.h"
#include "driver/gpio.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <mutex>
#include "esp_rom_sys.h"

namespace {
constexpr int kBatteryWarmupSamples = 1;
constexpr int kBatterySampleDelayUs = 250;
constexpr int kBatteryStartupDirectSamples = 3;
constexpr float kBatteryVoltageFilterAlpha = 0.50f;

float estimate_adc_full_scale_voltage(adc_atten_t atten) {
    switch (atten) {
        case ADC_ATTEN_DB_0:
            return 0.95f;
        case ADC_ATTEN_DB_2_5:
            return 1.25f;
        case ADC_ATTEN_DB_6:
            return 1.75f;
#if defined(ADC_ATTEN_DB_11)
        case ADC_ATTEN_DB_11:
            return 3.10f;
#endif
#if defined(ADC_ATTEN_DB_12)
        case ADC_ATTEN_DB_12:
            return 3.10f;
#endif
        default:
            return 3.10f;
    }
}
}


// Constructor takes initial config structs and EventBus reference
BatteryService::BatteryService(const BatteryConfig& initialConfig, const SystemBehaviorConfig& initialBehavior, EventBus& bus) :
    m_config(initialConfig), // Store initial config copy
    m_behaviorConfig(initialBehavior), // Store initial behavior config copy
    m_eventBus(bus),
    m_adc_channel(ADC_CHANNEL_0),
    m_adc_cali_handle(nullptr),
    m_adc_cali_enable(false),
    m_oneshot_adc_handle(nullptr),
    m_oversampling_count(64) // Initialize with default, will be overridden by applyConfig
{
    m_latest_status.percentage = 0;
    m_latest_status.voltage = 0.0f;
    m_latest_status.adcPinVoltage = 0.0f;
    m_latest_status.isLow = false;
    m_latest_status.isCritical = false;
    m_latest_status.adcCalibrated = false;
    applyConfig(initialConfig, initialBehavior); // Apply initial values
    ESP_LOGI(TAG, "BatteryService constructed.");
}

BatteryService::~BatteryService() {
    // --- Correct ADC Calibration Deinitialization ---
    if (m_adc_cali_enable && m_adc_cali_handle) {
        ESP_LOGI(TAG, "Deleting ADC calibration scheme handle.");
        // Choose the correct delete function based on the scheme created
        #if SOC_ADC_CALIBRATION_V1_SUPPORTED // Check if V1 (Curve Fitting) is supported
             ESP_LOGD(TAG,"Using V1 Calibration Scheme (Curve Fitting) for deinit.");
             ESP_ERROR_CHECK_WITHOUT_ABORT(adc_cali_delete_scheme_curve_fitting(m_adc_cali_handle));
        #elif SOC_ADC_CALIBRATION_V2_SUPPORTED // Check if V2 (Line Fitting) is supported
             ESP_LOGD(TAG,"Using V2 Calibration Scheme (Line Fitting) for deinit.");
             ESP_ERROR_CHECK_WITHOUT_ABORT(adc_cali_delete_scheme_line_fitting(m_adc_cali_handle));
        #endif
        m_adc_cali_handle = nullptr;
        m_adc_cali_enable = false;
    } else {
         ESP_LOGI(TAG, "ADC calibration handle already null or disabled.");
    }

    // --- Deinitialize ADC Unit ---
    if (m_oneshot_adc_handle) {
         ESP_LOGI(TAG, "Deleting ADC oneshot handle.");
         ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_del_unit(m_oneshot_adc_handle));
         m_oneshot_adc_handle = nullptr;
    }
     ESP_LOGI(TAG, "BatteryService Deinitialized.");
}

// Helper to map GPIO and check if it's valid for ADC1
esp_err_t BatteryService::map_gpio_to_channel() {
     adc_unit_t unit_id = ADC_UNIT_1;
     // Use the adc_pin from the member config struct
     esp_err_t ret = adc_oneshot_io_to_channel(m_config.adc_pin, &unit_id, &m_adc_channel);
     if (ret != ESP_OK) {
          ESP_LOGE(TAG, "GPIO %d is not a valid ADC1 pin (%s)", (int)m_config.adc_pin, esp_err_to_name(ret));
          m_adc_channel = (adc_channel_t)-1; // Mark channel as invalid
     } else {
          ESP_LOGI(TAG, "Mapped GPIO %d to ADC1 Channel %d", (int)m_config.adc_pin, (int)m_adc_channel);
     }
     return ret;
}


esp_err_t BatteryService::init() {
    ESP_LOGI(TAG, "Initializing BatteryService...");
    esp_err_t ret;

    // Config parameters already applied in constructor
    // loadConfigParameters();

    // 1. Map GPIO to Channel
    ret = map_gpio_to_channel();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to map GPIO to ADC channel");
    // Check if mapping was successful before proceeding
    if (m_adc_channel < 0) {
        ESP_LOGE(TAG, "Invalid ADC channel mapped, cannot proceed with ADC init.");
        return ESP_FAIL;
    }

    // 2. Init ADC Unit
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0) // Check if clk_src exists
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT, // Explicitly set default clock source if available
#endif
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&init_config1, &m_oneshot_adc_handle);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to init ADC1 oneshot unit");


    // 3. Configure ADC Channel (using values from m_config)
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = static_cast<adc_atten_t>(m_config.adc_atten),
        .bitwidth = static_cast<adc_bitwidth_t>(m_config.adc_bitwidth),
    };
    ret = adc_oneshot_config_channel(m_oneshot_adc_handle, m_adc_channel, &channel_config); // Use mapped channel
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to config ADC channel %d for GPIO %d", (int)m_adc_channel, (int)m_config.adc_pin);


    // 4. Initialize ADC Calibration
    m_adc_cali_enable = adc_calibration_init(m_adc_channel); // Pass mapped channel
    if (!m_adc_cali_enable) {
        ESP_LOGW(TAG, "ADC calibration skipped or failed. Voltage readings may be inaccurate.");
    }

    ESP_LOGI(TAG, "BatteryService Initialized (ADC GPIO: %d -> Channel: %d, Oversampling: %d).",
             (int)m_config.adc_pin, (int)m_adc_channel, m_oversampling_count);
    return ESP_OK;
}

// Updated Calibration Init for Oneshot Driver - Takes channel
bool BatteryService::adc_calibration_init(adc_channel_t channel) {
    esp_err_t ret = ESP_FAIL;
    m_adc_cali_enable = false;

    if (channel < 0) { // Check if channel is valid
        ESP_LOGE(TAG, "Cannot initialize calibration, invalid ADC channel.");
        return false;
    }

    ESP_LOGI(TAG, "Attempting ADC calibration for channel %d...", (int)channel);

    // Check which calibration scheme is supported and attempt creation
    #if SOC_ADC_CALIBRATION_V1_SUPPORTED
    ESP_LOGI(TAG, "Using ADC_CALI_SCHEME_CURVE_FITTING");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = channel,
        .atten = static_cast<adc_atten_t>(m_config.adc_atten),
        .bitwidth = static_cast<adc_bitwidth_t>(m_config.adc_bitwidth),
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &m_adc_cali_handle);
    if (ret == ESP_OK) {
        m_adc_cali_enable = true;
    }
    #else
    ESP_LOGW(TAG,"Curve Fitting V1 not supported by SOC_ADC_CALIBRATION. Check target.");
    ret = ESP_ERR_NOT_SUPPORTED;
    #endif

    // Handle results
    if (ret == ESP_OK && m_adc_cali_enable) {
        ESP_LOGI(TAG, "ADC Calibration scheme created successfully.");
    } else {
         if (ret == ESP_ERR_NOT_SUPPORTED) {
             ESP_LOGW(TAG,"ADC calibration scheme not supported for this configuration.");
         } else {
             ESP_LOGE(TAG,"Failed to create ADC calibration scheme: %s (%d)", esp_err_to_name(ret), ret);
         }
         m_adc_cali_handle = nullptr; // Ensure handle is null if creation failed
         m_adc_cali_enable = false;
    }
    return m_adc_cali_enable;
}

// Apply config values from structs
void BatteryService::applyConfig(const BatteryConfig& batConf, const SystemBehaviorConfig& behaviorConf) {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    const bool adc_path_changed =
        m_config.adc_pin != batConf.adc_pin ||
        m_config.adc_atten != batConf.adc_atten ||
        m_config.adc_bitwidth != batConf.adc_bitwidth ||
        std::abs(m_config.voltage_divider_ratio - batConf.voltage_divider_ratio) > 0.0001f;

    // Update local config struct copies
    m_config = batConf;
    m_behaviorConfig = behaviorConf;

    // Update relevant parameters
    m_oversampling_count = std::max(1, m_behaviorConfig.battery_oversampling_count); // Ensure at least 1 sample
    // Note: Read interval is now passed to the task constructor, not used directly here

    if (adc_path_changed) {
        m_filtered_adc_pin_voltage = -1.0f;
        m_filter_sample_count = 0;
    }

    ESP_LOGI(TAG, "Applied BatteryService params: Oversampling=%d, Vmin=%.2f, Vmax=%.2f, Ratio=%.2f, CritShutdown=%s",
             m_oversampling_count, m_config.voltage_min, m_config.voltage_max, m_config.voltage_divider_ratio,
             m_config.critical_battery_motor_shutdown_enabled ? "true" : "false");

    // Re-configure ADC channel if attenuation or bitwidth changed?
    // This requires more complex handling (deleting old calibration, re-init channel, re-init calib)
    // For now, assume these don't change frequently or require a restart.
    // If needed: check if m_oneshot_adc_handle exists, compare new config values, and call relevant ADC/Cali functions.
}

void BatteryService::handleEvent(const BaseEvent& event) {
    if (event.is<CONFIG_BatteryConfigUpdate>()) {
        handleBatteryConfigUpdate(event.as<CONFIG_BatteryConfigUpdate>());
    } else if (event.is<CONFIG_BehaviorConfigUpdate>()) {
        handleBehaviorConfigUpdate(event.as<CONFIG_BehaviorConfigUpdate>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

void BatteryService::handleBatteryConfigUpdate(const CONFIG_BatteryConfigUpdate& event) {
    ESP_LOGD(TAG, "Battery config update received.");
    SystemBehaviorConfig behaviorConfig;
    {
        std::lock_guard<std::mutex> lock(m_status_mutex);
        behaviorConfig = m_behaviorConfig;
    }
    applyConfig(event.config, behaviorConfig);
}

void BatteryService::handleBehaviorConfigUpdate(const CONFIG_BehaviorConfigUpdate& event) {
    ESP_LOGD(TAG, "Behavior config update received for BatteryService.");
    BatteryConfig batteryConfig;
    {
        std::lock_guard<std::mutex> lock(m_status_mutex);
        batteryConfig = m_config;
    }
    applyConfig(batteryConfig, event.config);
}

BatteryStatus BatteryService::getLatestStatus() const {
     std::lock_guard<std::mutex> lock(m_status_mutex);
     return m_latest_status;
}

void BatteryService::updateBatteryStatus() {
    BatteryConfig config;
    int oversampling_count = 1;
    {
        std::lock_guard<std::mutex> lock(m_status_mutex);
        config = m_config;
        oversampling_count = m_oversampling_count;
    }

    // Use the member variable for oversampling count
    if (!m_oneshot_adc_handle || m_adc_channel < 0) {
        ESP_LOGW(TAG, "ADC not initialized properly, cannot read battery voltage.");
        return;
    }

    for (int i = 0; i < kBatteryWarmupSamples; i++) {
        int ignored_raw_value = 0;
        esp_err_t ret = adc_oneshot_read(m_oneshot_adc_handle, m_adc_channel, &ignored_raw_value);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ADC warmup sample failed (%s).", esp_err_to_name(ret));
        }
        esp_rom_delay_us(kBatterySampleDelayUs);
    }

    m_raw_samples.clear();
    if (m_raw_samples.capacity() < static_cast<size_t>(oversampling_count)) {
        m_raw_samples.reserve(oversampling_count);
    }

    for (int i = 0; i < oversampling_count; i++) {
        int raw_value;
        esp_err_t ret = adc_oneshot_read(m_oneshot_adc_handle, m_adc_channel, &raw_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed (%s)! Skipping sample %d.", esp_err_to_name(ret), i);
            continue; // Skip this sample
        }
        m_raw_samples.push_back(raw_value);
        // Optionally add small delay between samples if ADC needs it
        if (i < oversampling_count - 1) {
             esp_rom_delay_us(kBatterySampleDelayUs);
        }
    }

    if (m_raw_samples.empty()) {
        ESP_LOGE(TAG, "No successful ADC samples obtained after %d attempts.", oversampling_count);
        return; // Cannot calculate voltage
    }

    int trim_count = 0;
    if (m_raw_samples.size() >= 8) {
        trim_count = static_cast<int>(m_raw_samples.size() / 8); // Trim 12.5% on each side.
    }
    const int start_index = trim_count;
    const int end_index = static_cast<int>(m_raw_samples.size()) - trim_count;

    auto samples_begin = m_raw_samples.begin();
    auto samples_end = m_raw_samples.end();
    auto trimmed_begin = samples_begin + start_index;
    auto trimmed_end = samples_begin + end_index;
    if (trim_count > 0) {
        std::nth_element(samples_begin, trimmed_begin, samples_end);
        std::nth_element(trimmed_begin, trimmed_end, samples_end);
    }

    int64_t raw_sum = 0;
    for (auto it = trimmed_begin; it != trimmed_end; ++it) {
        raw_sum += *it;
    }

    const int filtered_sample_count = std::max(1, end_index - start_index);
    int adc_raw = static_cast<int>(raw_sum / filtered_sample_count);
    ESP_LOGD(TAG, "Raw ADC value: %d (trimmed average of %d/%d samples)", adc_raw, filtered_sample_count, static_cast<int>(m_raw_samples.size()));

    // Convert to voltage
    const float adc_full_scale_voltage = estimate_adc_full_scale_voltage(static_cast<adc_atten_t>(config.adc_atten));
    float adc_pin_voltage = 0.0f;
    if (m_adc_cali_enable && m_adc_cali_handle) {
        int volt_mv;
        esp_err_t ret = adc_cali_raw_to_voltage(m_adc_cali_handle, adc_raw, &volt_mv);
        if (ret == ESP_OK) {
            adc_pin_voltage = volt_mv / 1000.0f; // Convert mV to V
        } else {
            ESP_LOGW(TAG, "ADC calibration conversion failed (%s)! Falling back.", esp_err_to_name(ret));
            adc_pin_voltage = adc_raw * (adc_full_scale_voltage / 4095.0f);
        }
    } else {
        // Manual conversion if calibration disabled (less accurate)
        adc_pin_voltage = adc_raw * (adc_full_scale_voltage / 4095.0f);
        ESP_LOGV(TAG, "ADC calibration disabled or failed, using raw estimate.");
    }

    {
        std::lock_guard<std::mutex> lock(m_status_mutex);
        if (m_filtered_adc_pin_voltage < 0.0f || m_filter_sample_count < kBatteryStartupDirectSamples) {
            m_filtered_adc_pin_voltage = adc_pin_voltage;
        } else {
            m_filtered_adc_pin_voltage =
                (kBatteryVoltageFilterAlpha * m_filtered_adc_pin_voltage) +
                ((1.0f - kBatteryVoltageFilterAlpha) * adc_pin_voltage);
        }
        m_filter_sample_count++;
        adc_pin_voltage = m_filtered_adc_pin_voltage;
    }

    float voltage = adc_pin_voltage;
    if (config.voltage_divider_ratio > 0.0f) {
        voltage = adc_pin_voltage * config.voltage_divider_ratio;
    }

    ESP_LOGD(TAG, "ADC pin voltage: %.3fV, battery voltage: %.3fV (filtered)", adc_pin_voltage, voltage);

    // Sanity check - clamp voltage to reasonable range
    if (voltage < 0.1f) {
        ESP_LOGW(TAG, "Invalid voltage reading (%.2fV), clamping to 0.1V", voltage);
        voltage = 0.1f;
    } else if (voltage > 20.0f) { // Assume no battery > 20V for this robot
        ESP_LOGW(TAG, "Unreasonable voltage reading (%.2fV), clamping to 20.0V", voltage);
        voltage = 20.0f;
    }

    // Calculate percentage based on min/max from config
    float percentage = 0.0f;
    if (voltage <= config.voltage_min) {
        percentage = 0.0f;
    } else if (voltage >= config.voltage_max) {
        percentage = 100.0f;
    } else {
        // Ensure denominator is not zero
        float range = config.voltage_max - config.voltage_min;
        if (range > 0.01f) { // Avoid division by zero or tiny ranges
             percentage = ((voltage - config.voltage_min) / range) * 100.0f;
        } else {
            percentage = (voltage >= config.voltage_max) ? 100.0f : 0.0f;
        }
    }

    // Clamp percentage just in case
    percentage = std::max(0.0f, std::min(100.0f, percentage));
    int rounded_percentage = static_cast<int>(percentage + 0.5f); // Round to nearest integer

    const float low_warning_voltage = config.voltage_min + (config.voltage_max - config.voltage_min) * 0.1f;
    const bool is_critical = voltage <= config.voltage_min;
    const bool is_low = voltage <= low_warning_voltage;

    // Create battery status struct
    BatteryStatus new_status;
    new_status.voltage = voltage;
    new_status.adcPinVoltage = adc_pin_voltage;
    new_status.percentage = rounded_percentage;
    new_status.isLow = is_low;
    new_status.isCritical = is_critical;
    new_status.adcCalibrated = m_adc_cali_enable && m_adc_cali_handle != nullptr;

    // Check if status has changed significantly before publishing
    bool status_changed = false;
    {
        std::lock_guard<std::mutex> lock(m_status_mutex);
        // Compare with more tolerance for voltage to avoid spamming events
        if (m_latest_status.percentage != rounded_percentage ||
            std::abs(m_latest_status.voltage - voltage) > 0.05f || // 50mV change threshold
            std::abs(m_latest_status.adcPinVoltage - adc_pin_voltage) > 0.02f ||
            m_latest_status.isLow != is_low ||
            m_latest_status.isCritical != is_critical ||
            m_latest_status.adcCalibrated != new_status.adcCalibrated) {

            status_changed = true;
        }

        m_latest_status = new_status; // Always keep telemetry and /api/state current.
    }

    // Publish event if status changed
    if (status_changed) {
        const char* battery_state = is_critical ? "CRITICAL" : (is_low ? "LOW" : "OK");
        ESP_LOGI(TAG, "Battery status updated: batt=%.2fV adc=%.2fV, %d%%, %s",
                voltage, adc_pin_voltage, rounded_percentage, battery_state);

        BATTERY_StatusUpdate event(new_status);
        m_eventBus.publish(event);
    } else {
        const char* battery_state = is_critical ? "CRITICAL" : (is_low ? "LOW" : "OK");
        ESP_LOGD(TAG, "Battery status unchanged: batt=%.2fV adc=%.2fV, %d%%, %s",
                voltage, adc_pin_voltage, rounded_percentage, battery_state);
    }
}

// EventHandler implementation
