#pragma once
#include "interfaces/IConfigObserver.hpp"

class IMPU6050Manager : public IConfigObserver {
    public:
        virtual float calculatePitch(float&, float) const = 0; 
        virtual float calculateFifoPitch(float&) const = 0;
        virtual ~IMPU6050Manager() = default;   
};
