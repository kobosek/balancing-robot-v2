#pragma once

#include "interfaces/IPIDController.hpp"
#include "esp_log.h"

enum PIDControllerType { ANGLE, SPEED };

class PIDController : public IPIDController {
public:
    PIDController(PIDControllerType p_type) : m_type(p_type), m_kp(0), m_ki(0), m_kd(0),
      m_outputMin(0), m_outputMax(0), m_iTermMin(0), m_iTermMax(0) {}

    esp_err_t init(const IRuntimeConfig&) override;
    esp_err_t onConfigUpdate(const IRuntimeConfig&) override;

    float compute(float, float&, float&, float, float) const override;
private:
    static constexpr const char* TAG = "PIDController";
    
    PIDControllerType m_type;

    esp_err_t setParams(const IRuntimeConfig&);
    void setKp(float);
    void setKi(float);
    void setKd(float);
    void setOutputLimits(float, float);
    void setItermLimits(float, float);

    float m_kp, m_ki, m_kd;
    float m_outputMin, m_outputMax;
    float m_iTermMin, m_iTermMax;
};
