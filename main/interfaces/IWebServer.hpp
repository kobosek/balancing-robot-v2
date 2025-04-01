#pragma once
#include "interfaces/IConfigObserver.hpp"

class IWebServer : public IConfigObserver {
    public:
        virtual void update_telemetry(float, float, float, float) = 0;
        virtual ~IWebServer() = default;
};