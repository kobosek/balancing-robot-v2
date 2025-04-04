// main/include/BalancingController.hpp
#pragma once

#include "freertos/FreeRTOS.h" // For TickType_t
#include "freertos/queue.h"    // For QueueHandle_t

// Forward declare interfaces if needed, or include directly
#include "interfaces/IMPU6050Manager.hpp"
#include "interfaces/IMotorDriver.hpp"
#include "interfaces/IEncoder.hpp"
#include "interfaces/IPIDController.hpp"
#include "interfaces/IWebServer.hpp" // Needed for TelemetryDataPoint definition

// Define buffer size here or get from a central config
#define CONTROLLER_ERROR_BUFFER_SIZE 50

class BalancingController {
public:
    // Constructor takes dependencies and configuration
    BalancingController(
        IMPU6050Manager& mpuManager,
        IMotorDriver& motorL,
        IMotorDriver& motorR,
        IEncoder& encoderL,
        IEncoder& encoderR,
        IPIDController& anglePid,
        IPIDController& speedPidL,
        IPIDController& speedPidR,
        QueueHandle_t telemetryQ, // Pass the queue handle directly
        int loopIntervalMs
    );

    // Performs one iteration of the control loop
    void runCycle();

    // Reset state if needed (e.g., after safety stop)
    void resetState();

private:
    // Component References (Dependencies)
    IMPU6050Manager& m_mpu6050Manager;
    IMotorDriver& m_motorLeft;
    IMotorDriver& m_motorRight;
    IEncoder& m_encoderLeft;
    IEncoder& m_encoderRight;
    IPIDController& m_anglePidController;
    IPIDController& m_speedPidControllerLeft;
    IPIDController& m_speedPidControllerRight;
    QueueHandle_t m_telemetryQueue;

    // Configuration
    int m_loopIntervalMs;

    // State Variables (Moved from ControlContext)
    float m_pitch = 0.0f;
    float m_angleSetPoint = 0.0f; // Could be made configurable
    float m_lastSpeedSetPoint = 0.0f; // Still needed? Maybe only for RMSE?
    float m_speedSetPoint = 0.0f;
    float m_currentSpeedLeft = 0.0f;
    float m_dutyLeft = 0.0f;
    float m_currentSpeedRight = 0.0f;
    float m_dutyRight = 0.0f;
    int64_t m_lastTimeMPU6050 = 0;
    int64_t m_lastTimeLeftMotor = 0;
    int64_t m_lastTimeRightMotor = 0;
    float m_speedErrorBufferLeft[CONTROLLER_ERROR_BUFFER_SIZE] = {0};
    float m_speedErrorBufferRight[CONTROLLER_ERROR_BUFFER_SIZE] = {0};
    int m_errorBufferIndex = 0;
    float m_lastSpeedSetPointForRMSE = 0.0f;

    // Helper function (could be static if doesn't access members)
    float calculateRMSE(float* errorBuffer, int bufferSize) const;

    // Logging Tag
    static constexpr const char* TAG = "BalancingController";
};