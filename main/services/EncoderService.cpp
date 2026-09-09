// main/EncoderService.cpp
#include "EncoderService.hpp"           // Relative path within module's include dir
#include "esp_check.h"
#include <cmath>
#include <algorithm>
#include "esp_timer.h"
#include "esp_log.h"                    // Moved from header

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

EncoderService::EncoderService(const EncoderConfig& config, int64_t nominalPeriodUs) :
    m_config(config),
    m_nominalPeriodUs(std::max<int64_t>(1, nominalPeriodUs)),
    m_unit_left(nullptr),
    m_unit_right(nullptr)
{
    // Pre-calculate conversion factor: (DEG/rev) / (pulses/rev_motor) / gear_ratio
    // DEG/rev = 360
    if (m_config.pulses_per_revolution_motor > 0 && m_config.gear_ratio > 0) {
        m_degs_per_pulse = 360.0f / (m_config.pulses_per_revolution_motor * m_config.gear_ratio); // <-- Changed calculation
        ESP_LOGI(TAG,"Encoder degrees per pulse calculated: %f", m_degs_per_pulse);
    } else {
        ESP_LOGE(TAG,"Invalid encoder config: pulses/rev or gear ratio is zero!");
        m_degs_per_pulse = 0.0f;
    }
    if (m_config.speed_filter_alpha > 0 && m_config.speed_filter_alpha < 1)
        m_filterLogRetention = std::log1p(-m_config.speed_filter_alpha);
    reset();
}

EncoderService::~EncoderService() {
    if (m_unit_left) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_stop(m_unit_left));
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_disable(m_unit_left));
    }
    if (m_unit_right) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_stop(m_unit_right));
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_disable(m_unit_right));
    }
    if (m_channel_left_a) { pcnt_del_channel(m_channel_left_a); }
    if (m_channel_left_b) { pcnt_del_channel(m_channel_left_b); }
    if (m_channel_right_a) { pcnt_del_channel(m_channel_right_a); }
    if (m_channel_right_b) { pcnt_del_channel(m_channel_right_b); }
    if (m_unit_left) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_remove_watch_point(m_unit_left, m_config.pcnt_low_limit));
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_remove_watch_point(m_unit_left, m_config.pcnt_high_limit));
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_del_unit(m_unit_left));
    }
    if (m_unit_right) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_remove_watch_point(m_unit_right, m_config.pcnt_low_limit));
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_remove_watch_point(m_unit_right, m_config.pcnt_high_limit));
        ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_del_unit(m_unit_right));
    }
}

EncoderFrame EncoderService::getFrame() const {
    portENTER_CRITICAL(&m_frameMux);
    const auto frame = m_frame;
    portEXIT_CRITICAL(&m_frameMux);
    return frame;
}

void EncoderService::publish(EncoderFrame frame) {
    portENTER_CRITICAL(&m_frameMux);
    frame.sequence = m_frame.sequence + 1;
    m_frame = frame;
    portEXIT_CRITICAL(&m_frameMux);
}

void EncoderService::reset() {
    std::lock_guard<std::mutex> lock(m_writerMutex);
    // Seed from the real accumulated count; do not discard edges in hardware.
    m_left.seeded = m_right.seeded = false;
    m_left.logicalCount = m_right.logicalCount = 0;
    m_left.speedDps = m_right.speedDps = 0;
    ++m_left.continuityEpoch;
    ++m_right.continuityEpoch;
    m_left.continuityLossActive = m_right.continuityLossActive = false;
    EncoderFrame frame;
    frame.left = readWheel(m_unit_left, m_left);
    frame.right = readWheel(m_unit_right, m_right);
    frame.sampleTimestampUs = esp_timer_get_time();
    publish(frame);
}

esp_err_t EncoderService::init() {
    ESP_LOGI(TAG, "Initializing EncoderService...");
    esp_err_t ret;
    ret = initPCNTUnit(m_config.left_pin_a, m_config.left_pin_b, &m_unit_left, &m_channel_left_a, &m_channel_left_b);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Left Encoder PCNT");
    ESP_LOGI(TAG, "Left Encoder PCNT Initialized (Pins A:%d, B:%d)", m_config.left_pin_a, m_config.left_pin_b);
    ret = initPCNTUnit(m_config.right_pin_a, m_config.right_pin_b, &m_unit_right, &m_channel_right_a, &m_channel_right_b);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed init Right Encoder PCNT");
    ESP_LOGI(TAG, "Right Encoder PCNT Initialized (Pins A:%d, B:%d)", m_config.right_pin_a, m_config.right_pin_b);
    reset();
    ESP_LOGI(TAG, "EncoderService Initialized Successfully.");
    return ESP_OK;
}

esp_err_t EncoderService::initPCNTUnit(int pinA, int pinB, pcnt_unit_handle_t* unit_handle,
                                       pcnt_channel_handle_t* channel_a_handle,
                                       pcnt_channel_handle_t* channel_b_handle) {
     ESP_LOGD(TAG, "Init PCNT Unit for pins A:%d, B:%d", pinA, pinB);
     *unit_handle = nullptr;
     *channel_a_handle = nullptr;
     *channel_b_handle = nullptr;
     pcnt_unit_handle_t unit = nullptr;
     pcnt_channel_handle_t pcnt_chan_a = nullptr;
     pcnt_channel_handle_t pcnt_chan_b = nullptr;
     bool unit_enabled = false;
     bool unit_started = false;
     bool low_watch = false, high_watch = false;
     auto cleanup = [&]() {
         if (unit_started) {
             ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_stop(unit));
         }
         if (unit_enabled) {
             ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_disable(unit));
         }
         if (low_watch) ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_remove_watch_point(unit, m_config.pcnt_low_limit));
         if (high_watch) ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_remove_watch_point(unit, m_config.pcnt_high_limit));
         if (pcnt_chan_a) {
             ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_del_channel(pcnt_chan_a));
         }
         if (pcnt_chan_b) {
             ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_del_channel(pcnt_chan_b));
         }
         if (unit) {
             ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_del_unit(unit));
         }
     };
     pcnt_unit_config_t unit_config{};
     unit_config.low_limit = m_config.pcnt_low_limit;
     unit_config.high_limit = m_config.pcnt_high_limit;
     unit_config.flags.accum_count = 1;
     esp_err_t ret = pcnt_new_unit(&unit_config, &unit);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed create PCNT unit: %s", esp_err_to_name(ret));
         return ret;
     }
     pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = (uint32_t)m_config.pcnt_filter_ns };
     ret = pcnt_unit_set_glitch_filter(unit, &filter_config);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed set PCNT glitch filter: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     pcnt_chan_config_t chan_a_config = { .edge_gpio_num = pinA, .level_gpio_num = pinB, .flags = {} };
     ret = pcnt_new_channel(unit, &chan_a_config, &pcnt_chan_a);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed create PCNT channel A: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     pcnt_chan_config_t chan_b_config = { .edge_gpio_num = pinB, .level_gpio_num = pinA, .flags = {} };
     ret = pcnt_new_channel(unit, &chan_b_config, &pcnt_chan_b);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed create PCNT channel B: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Chan A edge fail: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Chan A level fail: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Chan B edge fail: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Chan B level fail: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     ret = pcnt_unit_add_watch_point(unit, m_config.pcnt_low_limit);
     if (ret != ESP_OK) { cleanup(); return ret; }
     low_watch = true;
     ret = pcnt_unit_add_watch_point(unit, m_config.pcnt_high_limit);
     if (ret != ESP_OK) { cleanup(); return ret; }
     high_watch = true;
     // Clear activates watch points and resets the software accumulator.
     ret = pcnt_unit_clear_count(unit);
     if (ret != ESP_OK) { cleanup(); return ret; }
     ret = pcnt_unit_enable(unit);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed enable PCNT unit: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     unit_enabled = true;
     ret = pcnt_unit_start(unit);
     if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed start PCNT unit: %s", esp_err_to_name(ret)); cleanup(); return ret; }
     unit_started = true;
     *unit_handle = unit;
     *channel_a_handle = pcnt_chan_a;
     *channel_b_handle = pcnt_chan_b;
     return ESP_OK;
}


EncoderWheelFrame EncoderService::readWheel(pcnt_unit_handle_t unit, WheelState& state) {
    EncoderWheelFrame frame;
    frame.logicalCount = state.logicalCount;
    frame.continuityLost = state.continuityLost;
    frame.continuityEpoch = state.continuityEpoch;
    if (!unit || m_degs_per_pulse <= 0) { frame.error = ESP_ERR_INVALID_STATE; return frame; }
    const auto markContinuityLoss = [&]() {
        if (!state.continuityLossActive) {
            ++state.continuityEpoch;
            state.continuityLossActive = true;
        }
        state.continuityLost = true;
        frame.continuityLost = true;
        frame.continuityEpoch = state.continuityEpoch;
    };
    // Complete a stopped rebase before accepting another measurement. On any
    // error stay invalid and retry; never invent a valid zero-speed sample.
    if (state.stopped) {
        frame.error = pcnt_unit_clear_count(unit);
        if (frame.error == ESP_OK) frame.error = pcnt_unit_start(unit);
        if (frame.error != ESP_OK) {
            markContinuityLoss();
            return frame;
        }
        state.stopped = false;
        state.seeded = false;
    }
    int count = 0;
    const int64_t jumpLimit = std::min(-m_config.pcnt_low_limit, m_config.pcnt_high_limit) / 2;
    frame.sampleTimestampUs = esp_timer_get_time();
    frame.error = pcnt_unit_get_count(unit, &count);
    if (frame.error == ESP_OK && state.seeded &&
        std::abs(static_cast<int64_t>(count) - state.previousCount) >= jumpLimit) {
        // The hardware can reach zero before the other core's PCNT ISR adds
        // the limit to the accumulator. Re-observe once, without waiting or
        // correcting the count. A persistent discontinuity still invalidates.
        frame.sampleTimestampUs = esp_timer_get_time();
        frame.error = pcnt_unit_get_count(unit, &count);
    }
    if (frame.error != ESP_OK) {
        state.seeded = false;
        state.speedDps = 0;
        markContinuityLoss();
        return frame;
    }
    frame.rawCount = count;
    const int64_t elapsed = frame.sampleTimestampUs - state.previousTimestampUs;
    const int64_t delta = static_cast<int64_t>(count) - state.previousCount;
    frame.measurementPeriodUs = state.seeded ? elapsed : 0;
    // No wrap correction: IDF adds the limit in its ISR. A half-limit jump
    // cannot be trusted (including an observation before that ISR runs).
    const bool trustedDelta = state.seeded && elapsed > 0 && std::abs(delta) < jumpLimit;
    if (trustedDelta) {
        frame.deltaCount = delta;
        state.logicalCount += delta;
        const float speed = delta * m_degs_per_pulse * 1000000.0f / elapsed;
        // Preserve the configured response at the nominal period, and the same
        // decay per unit time under jitter. Fixed alpha amplified short samples.
        float alpha = m_config.speed_filter_alpha;
        if (alpha > 0 && alpha < 1 && elapsed != m_nominalPeriodUs)
            alpha = -std::expm1(m_filterLogRetention *
                (static_cast<float>(elapsed) / m_nominalPeriodUs));
        state.speedDps = alpha * speed + (1.0f - alpha) * state.speedDps;
        frame.valid = std::isfinite(state.speedDps);
        frame.speedDps = frame.valid ? state.speedDps : 0;
        if (frame.valid) state.continuityLossActive = false;
    } else {
        state.speedDps = 0;
        if (state.seeded) markContinuityLoss();
    }
    state.previousCount = count;
    state.previousTimestampUs = frame.sampleTimestampUs;
    state.seeded = true;
    frame.logicalCount = state.logicalCount;
    if (std::abs(static_cast<int64_t>(count)) >= REBASE_THRESHOLD) {
        frame.valid = false;
        frame.speedDps = 0;
        frame.rebased = true;
        markContinuityLoss();
        frame.error = pcnt_unit_stop(unit);
        if (frame.error == ESP_OK) state.stopped = true;
        // Stop failure must not leave an overflowing accumulator running
        // unnoticed: every subsequent frame remains invalid and retries stop.
    }
    frame.continuityLost = state.continuityLost;
    frame.continuityEpoch = state.continuityEpoch;
    return frame;
}

void EncoderService::update() {
    std::lock_guard<std::mutex> lock(m_writerMutex);
    const auto previous = getFrame();
    const auto now = esp_timer_get_time();
    // Delay-until catch-up iterations can be microseconds apart. Accumulate
    // their pulses into a useful interval rather than amplify one edge by 1/dt.
    if (previous.sampleTimestampUs > 0 && now >= previous.sampleTimestampUs &&
        now - previous.sampleTimestampUs < 1000) return;
    EncoderFrame frame;
    frame.left = readWheel(m_unit_left, m_left);
    frame.right = readWheel(m_unit_right, m_right);
    frame.sampleTimestampUs = esp_timer_get_time();
    publish(frame);
}
