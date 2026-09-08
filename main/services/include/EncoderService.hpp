#pragma once

#include "config/EncoderConfig.hpp"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include <cstdint>
#include <mutex>

struct EncoderWheelFrame {
    int32_t rawCount = 0; // ESP-IDF accumulated count, not the hardware register.
    int64_t logicalCount = 0;
    int64_t deltaCount = 0;
    int64_t sampleTimestampUs = 0;
    int64_t measurementPeriodUs = 0;
    float speedDps = 0;
    bool valid = false;
    bool rebased = false;
    bool continuityLost = false; // Sticky: edges during a stopped rebase are unknown.
    esp_err_t error = ESP_OK;
};

struct EncoderFrame {
    uint64_t sequence = 0;
    int64_t sampleTimestampUs = 0;
    EncoderWheelFrame left, right;
};

class EncoderService {
public:
    explicit EncoderService(const EncoderConfig& config, int64_t nominalPeriodUs = 5000);
    ~EncoderService();
    esp_err_t init();
    void update();
    void reset(); // Logical baseline only; never clears a running hardware counter.
    EncoderFrame getFrame() const;
    float getLeftSpeedDegPerSec() const { return getFrame().left.speedDps; }
    float getRightSpeedDegPerSec() const { return getFrame().right.speedDps; }

private:
    static constexpr const char* TAG = "EncoderService";
    // Leave ample headroom for the SDK's signed 32-bit accumulator.
    static constexpr int32_t REBASE_THRESHOLD = 1 << 28;
    const EncoderConfig m_config;
    const int64_t m_nominalPeriodUs;
    float m_filterLogRetention = 0;
    pcnt_unit_handle_t m_unit_left = nullptr, m_unit_right = nullptr;
    pcnt_channel_handle_t m_channel_left_a = nullptr, m_channel_left_b = nullptr;
    pcnt_channel_handle_t m_channel_right_a = nullptr, m_channel_right_b = nullptr;
    struct WheelState {
        int32_t previousCount = 0;
        int64_t previousTimestampUs = 0, logicalCount = 0;
        float speedDps = 0;
        bool seeded = false, stopped = false, continuityLost = false;
    };
    WheelState m_left, m_right;
    float m_degs_per_pulse = 0;
    std::mutex m_writerMutex;
    mutable portMUX_TYPE m_frameMux = portMUX_INITIALIZER_UNLOCKED;
    EncoderFrame m_frame;
    void publish(EncoderFrame frame);
    EncoderWheelFrame readWheel(pcnt_unit_handle_t unit, WheelState& state);
    esp_err_t initPCNTUnit(int pinA, int pinB, pcnt_unit_handle_t* unit,
                          pcnt_channel_handle_t* channelA, pcnt_channel_handle_t* channelB);
};
