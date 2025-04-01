#pragma once
#include "interfaces/IConfigObserver.hpp"

class IEncoder : public IConfigObserver {
    public:
        virtual float getSpeed(float dt) = 0;
        virtual ~IEncoder() = default;
};