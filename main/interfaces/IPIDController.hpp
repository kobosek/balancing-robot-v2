#pragma once

#include "interfaces/IConfigObserver.hpp"

class IPIDController : public IConfigObserver {
  public:
    virtual float compute(float, float&, float&, float, float) const = 0;
    virtual ~IPIDController() = default;
};