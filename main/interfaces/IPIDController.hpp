#pragma once

#include "interfaces/IConfigObserver.hpp"
#include "interfaces/IRuntimeConfig.hpp"

class IPIDController : public IConfigObserver {
  public:
    virtual esp_err_t init(const PIDConfig& config) = 0;
    virtual esp_err_t onConfigUpdate(const PIDConfig& config) = 0;

    // Compute modifies internal state now, so non-const
    virtual float compute(float setpoint, float currentValue, float dt) = 0;
    virtual void reset() = 0; // Add a reset method interface

    // Remove old init/onConfigUpdate taking IRuntimeConfig
    virtual esp_err_t init(const IRuntimeConfig&) override { return ESP_ERR_NOT_SUPPORTED; };
    virtual esp_err_t onConfigUpdate(const IRuntimeConfig&) override { return ESP_ERR_NOT_SUPPORTED; };
    virtual ~IPIDController() = default;
};