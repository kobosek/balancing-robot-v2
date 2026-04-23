#pragma once

#include "esp_err.h"

class ApplicationContext;

class ApplicationEventWiring {
public:
    esp_err_t wire(const ApplicationContext& context) const;
};
