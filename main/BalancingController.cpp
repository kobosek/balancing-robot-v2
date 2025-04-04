// main/BalancingController.cpp
#include "include/BalancingController.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath> // For std::sqrt, std::pow

// Constructor Implementation
BalancingController::BalancingController(
    IMPU6050Manager& mpuManager,
    IMotorDriver& motorL,
    IMotorDriver& motorR,
    IEncoder& encoderL,
    IEncoder& encoderR,
    IPIDController& anglePid,
    IPIDController& speedPidL,
    IPIDController& speedPidR,
    QueueHandle_t telemetryQ,
    int loopIntervalMs
) : // Initializer list for references and constants
    m_mpu6050Manager(mpuManager),
    m_motorLeft(motorL),
    m_motorRight(motorR),
    m_encoderLeft(encoderL),
    m_encoderRight(encoderR),
    m_anglePidController(anglePid),
    m_speedPidControllerLeft(speedPidL),
    m_speedPidControllerRight(speedPidR),
    m_telemetryQueue(telemetryQ),
    m_loopIntervalMs(loopIntervalMs)
{
    // Initialize state variables
    int64_t initial_time = esp_timer_get_time();
    m_lastTimeMPU6050 = initial_time;
    m_lastTimeLeftMotor = initial_time;
    m_lastTimeRightMotor = initial_time;
    resetState(); // Initialize pitch, speeds, duties etc.

    ESP_LOGI(TAG, "BalancingController Initialized. Loop Interval: %d ms", m_loopIntervalMs);
}

// Reset internal state
void BalancingController::resetState() {
    m_pitch = 0.0f;
    // m_angleSetPoint = 0.0f; // Keep setpoint unless explicitly changed
    m_lastSpeedSetPoint = 0.0f;
    m_speedSetPoint = 0.0f;
    m_currentSpeedLeft = 0.0f;
    m_dutyLeft = 0.0f;
    m_currentSpeedRight = 0.0f;
    m_dutyRight = 0.0f;
    m_errorBufferIndex = 0;
    m_lastSpeedSetPointForRMSE = 0.0f;
    // Clear error buffers
    for(int i=0; i<CONTROLLER_ERROR_BUFFER_SIZE; ++i) {
        m_speedErrorBufferLeft[i] = 0.0f;
        m_speedErrorBufferRight[i] = 0.0f;
    }
    // Reset PIDs (important!)
    m_anglePidController.reset();
    m_speedPidControllerLeft.reset();
    m_speedPidControllerRight.reset();

    ESP_LOGI(TAG, "Controller state reset.");
}


// runCycle Method Implementation
void BalancingController::runCycle() {
    // --- Time Calculation ---
    int64_t currentTime = esp_timer_get_time();
    float dt_angle = (currentTime - m_lastTimeMPU6050) / 1000000.0f;
    m_lastTimeMPU6050 = currentTime;

    if (dt_angle <= 0) {
        ESP_LOGW(TAG, "Invalid dt_angle (%.6f), skipping cycle logic.", dt_angle);
        // The task calling this should handle the vTaskDelayUntil
        return;
    }

    // --- Get Pitch ---
    // Assumes calculateFifoPitch modifies m_pitch by reference
    m_mpu6050Manager.calculateFifoPitch(m_pitch);
    ESP_LOGV(TAG, "Current pitch: %.2f", m_pitch);

    // --- Angle PID ---
    m_speedSetPoint = m_anglePidController.compute(m_angleSetPoint, m_pitch, dt_angle);

    // --- Left Motor ---
    currentTime = esp_timer_get_time();
    float dt_speed_left = (currentTime - m_lastTimeLeftMotor) / 1000000.0f;
    m_lastTimeLeftMotor = currentTime;

    if (dt_speed_left <= 0) {
         ESP_LOGW(TAG, "Invalid dt_speed_left (%.6f), setting dutyLeft=0.", dt_speed_left);
         m_dutyLeft = 0;
         m_speedPidControllerLeft.reset(); // Reset PID if timing is bad
    } else {
        m_currentSpeedLeft = m_encoderLeft.getSpeed(dt_speed_left);
        m_dutyLeft = m_speedPidControllerLeft.compute(m_speedSetPoint, m_currentSpeedLeft, dt_speed_left);
    }
    m_motorLeft.setSpeed(m_dutyLeft);

    // --- Right Motor ---
    currentTime = esp_timer_get_time();
    float dt_speed_right = (currentTime - m_lastTimeRightMotor) / 1000000.0f;
    m_lastTimeRightMotor = currentTime;

    if(dt_speed_right <= 0) {
         ESP_LOGW(TAG, "Invalid dt_speed_right (%.6f), setting dutyRight=0.", dt_speed_right);
         m_dutyRight = 0;
         m_speedPidControllerRight.reset(); // Reset PID if timing is bad
    } else {
        m_currentSpeedRight = m_encoderRight.getSpeed(dt_speed_right);
        m_dutyRight = m_speedPidControllerRight.compute(m_speedSetPoint, m_currentSpeedRight, dt_speed_right);
    }
    m_motorRight.setSpeed(m_dutyRight);

    // --- RMSE Calculation ---
    float errorLeft = m_lastSpeedSetPointForRMSE - m_currentSpeedLeft;
    float errorRight = m_lastSpeedSetPointForRMSE - m_currentSpeedRight;
    m_speedErrorBufferLeft[m_errorBufferIndex] = errorLeft;
    m_speedErrorBufferRight[m_errorBufferIndex] = errorRight;
    float rmseSpeedErrorLeft = calculateRMSE(m_speedErrorBufferLeft, CONTROLLER_ERROR_BUFFER_SIZE);
    float rmseSpeedErrorRight = calculateRMSE(m_speedErrorBufferRight, CONTROLLER_ERROR_BUFFER_SIZE);
    m_errorBufferIndex = (m_errorBufferIndex + 1) % CONTROLLER_ERROR_BUFFER_SIZE;
    m_lastSpeedSetPointForRMSE = m_speedSetPoint;

    // --- Telemetry --- (Send to Queue)
    TelemetryDataPoint currentData = {
        .pitch = m_pitch,
        .desiredSpeed = m_speedSetPoint,
        .currentSpeedLeft = m_currentSpeedLeft,
        .currentSpeedRight = m_currentSpeedRight,
        .rmseLeft = rmseSpeedErrorLeft,
        .rmseRight = rmseSpeedErrorRight
    };
    // Send data to the queue - non-blocking
    if (m_telemetryQueue != NULL) { // Check if queue exists
        BaseType_t sendResult = xQueueSend(m_telemetryQueue, &currentData, (TickType_t)0);
        if (sendResult != pdPASS) {
            // Queue was full, data was dropped. Log sparingly.
            static uint64_t drop_counter = 0; // Static counter within the method scope? Or make member? Member better.
            drop_counter++;
            if (drop_counter % 100 == 1) {
                ESP_LOGW(TAG, "Telemetry queue full. Total dropped points approx: %llu", drop_counter);
            }
        } else {
             ESP_LOGV(TAG,"Sent telemetry point to queue.");
        }
    }
}

// Helper function implementation
float BalancingController::calculateRMSE(float* errorBuffer, int bufferSize) const {
    float sumSquaredError = 0;
    for(int i = 0; i < bufferSize; i++) {
        sumSquaredError += std::pow(errorBuffer[i], 2);
    }
    // Avoid division by zero if bufferSize is 0
    return bufferSize > 0 ? std::sqrt(sumSquaredError / bufferSize) : 0.0f;
}