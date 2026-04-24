#pragma once

#include "esp_err.h"
#include <cstdint>

enum class I2CErrorType {
    NONE = 0,
    TIMEOUT,
    NACK,
    BUS_ERROR,
    ARBITRATION_LOST,
    FIFO_OVERFLOW,
    PERMANENT_FAILURE,
    DEVICE_NOT_FOUND,
    INVALID_RESPONSE,
    HARDWARE_FAULT
};

enum class I2CErrorSeverity {
    NONE = 0,
    TEMPORARY,
    RECOVERABLE,
    PERMANENT
};

struct I2CStats {
    uint32_t total_operations = 0;
    uint32_t successful_operations = 0;

    uint32_t timeout_errors = 0;
    uint32_t nack_errors = 0;
    uint32_t bus_errors = 0;
    uint32_t arbitration_lost_errors = 0;
    uint32_t fifo_overflow_errors = 0;
    uint32_t permanent_failure_errors = 0;
    uint32_t device_not_found_errors = 0;
    uint32_t invalid_response_errors = 0;
    uint32_t hardware_fault_errors = 0;

    float current_error_rate_percent = 0.0f;
    uint32_t consecutive_failures = 0;
    uint32_t max_consecutive_failures = 0;

    uint64_t last_operation_timestamp = 0;
    uint32_t avg_operation_time_us = 0;
    uint32_t max_operation_time_us = 0;
};

class I2CHealthTracker {
public:
    I2CHealthTracker() = default;

    void reset();
    void recordSuccess();
    void recordFailure(esp_err_t error);

    I2CStats getStats() const { return m_stats; }
    I2CErrorType classifyError(esp_err_t error) const;
    I2CErrorSeverity classifySeverity(I2CErrorType errorType) const;

private:
    static uint64_t getCurrentTimestampUs();

    I2CStats m_stats{};
};
