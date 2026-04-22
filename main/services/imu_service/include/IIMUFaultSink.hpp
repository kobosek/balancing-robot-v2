#pragma once

#include "esp_err.h"

class IIMUFaultSink {
public:
    virtual ~IIMUFaultSink() = default;

    virtual void onIMUHardFault(esp_err_t errorCode) = 0;
};
