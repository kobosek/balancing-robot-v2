// main.cpp

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h" 
#include "esp_log.h"
#include "esp_pthread.h"

#include "include/ComponentHandler.hpp"
#include "include/RuntimeConfig.hpp"

#include "interfaces/IWebServer.hpp"
#include "interfaces/IPIDController.hpp"
#include "interfaces/IMotorDriver.hpp"
#include "interfaces/IMPU6050Manager.hpp"
#include "interfaces/IEncoder.hpp"
#include "math.h"

#define TAG "PID_DEBUG"

static const int ERROR_BUFFER_SIZE = 50;

struct ControlContext {
    const IMPU6050Manager& mpu6050Manager;
    const IMotorDriver& motorLeft;
    const IMotorDriver& motorRight;
    IEncoder& encoderLeft;
    IEncoder& encoderRight;
    const IPIDController& anglePidController;
    const IPIDController& speedPidController;
    IWebServer& webServer;
    float pitch;
    float angleSetPoint;
    float angleIntegral;
    float angleLastError;
    float lastSpeedSetPoint;
    float speedSetPoint;
    float speedLeftIntegral;
    float speedLeftLastError;
    float currentSpeedLeft;
    float dutyLeft;
    float speedRightIntegral;
    float speedRightLastError;
    float currentSpeedRight;
    float dutyRight;
    int64_t lastTimeMPU6050;
    int64_t lastTimeLeftMotor;
    int64_t lastTimeRightMotor;
    int64_t logCounter;
    float deltaSpeed;
    float speedErrorBuffer[ERROR_BUFFER_SIZE] = {0};
    int errorBufferIndex = 0;
};


float calculateRMSE(float* errorBuffer, int bufferSize) {
    float sumSquaredError = 0;
    for(int i = 0; i < bufferSize; i++) {
        sumSquaredError += pow(errorBuffer[i], 2);
    }
    return sqrt(sumSquaredError / bufferSize);
}

static void control_loop_callback(void* arg);

extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    // Create and start the PID control task
    RuntimeConfig config;
    config.init("/spiffs/config.json");
    ComponentHandler handler(config);
    handler.init();

    ControlContext context = {
        .mpu6050Manager = handler.getMPU6050Manager(),
        .motorLeft = handler.getMotorLeft(),
        .motorRight = handler.getMotorRight(),
        .encoderLeft = handler.getEncoderLeft(),
        .encoderRight = handler.getEncoderRight(),
        .anglePidController = handler.getAnglePIDController(),
        .speedPidController = handler.getSpeedPIDController(),
        .webServer = handler.getWebServer(),
        .pitch = 0.0f,
        .angleSetPoint = 0.0f,
        .angleIntegral = 0.0f,
        .angleLastError = 0.0f,
        .lastSpeedSetPoint = 0.0f,
        .speedSetPoint = 0.0f,
        .speedLeftIntegral = 0.0f,
        .speedLeftLastError = 0.0f,
        .currentSpeedLeft = 0.0f,
        .dutyLeft = 0.0f,
        .speedRightIntegral = 0.0f,
        .speedRightLastError = 0.0f,
        .currentSpeedRight = 0.0f,
        .dutyRight = 0.0f,
        .lastTimeMPU6050 = esp_timer_get_time(),
        .lastTimeLeftMotor = esp_timer_get_time(),
        .lastTimeRightMotor = esp_timer_get_time(),
        .logCounter = 0,
    };   

    const esp_timer_create_args_t timer_args = {
        .callback = &control_loop_callback,
        .arg = &context,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "control_loop"
    };
    
    esp_timer_handle_t control_loop_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &control_loop_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(control_loop_timer, 5 * 1000));

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000 ms
    }
}

static void control_loop_callback(void* arg) {
    ControlContext* ctx = (ControlContext*)arg;
    if (!ctx) {
        ESP_LOGE(TAG, "Control context is NULL!");
        return;
    }
        int64_t currentTime = esp_timer_get_time();
        float dt = (currentTime - ctx->lastTimeMPU6050) / 1000000.0f;
        ctx->lastTimeMPU6050 = currentTime;
        ctx->pitch = ctx->mpu6050Manager.calculateFifoPitch(ctx->pitch); 
        ctx->speedSetPoint = ctx->anglePidController.compute(ctx->angleSetPoint, ctx->angleIntegral, ctx->angleLastError, ctx->pitch, dt);      

        // //left motor
        currentTime = esp_timer_get_time();
        dt = (currentTime - ctx->lastTimeLeftMotor) / 1000000.0f;
        ctx->lastTimeLeftMotor = currentTime;

        ctx->currentSpeedLeft = ctx->encoderLeft.getSpeed(dt); //speed in deg/s of the outer shaft
        ctx->dutyLeft = ctx->speedPidController.compute(ctx->speedSetPoint, ctx->speedLeftIntegral, ctx->speedLeftLastError, ctx->currentSpeedLeft, dt);
        ctx->motorLeft.setSpeed(ctx->dutyLeft);
    
        ctx->speedErrorBuffer[ctx->errorBufferIndex] = ctx->lastSpeedSetPoint - ctx->currentSpeedLeft;
        ctx->lastSpeedSetPoint = ctx->speedSetPoint;
        ctx->errorBufferIndex = (ctx->errorBufferIndex + 1) % ERROR_BUFFER_SIZE;
        float rmseSpeedErrorLeft = calculateRMSE(ctx->speedErrorBuffer, ERROR_BUFFER_SIZE);

        //right motor
        currentTime = esp_timer_get_time();
        dt = (currentTime - ctx->lastTimeRightMotor) / 1000000.0f;
        ctx->lastTimeRightMotor = currentTime;

        ctx->currentSpeedRight = ctx->encoderRight.getSpeed(dt);//speed in deg/s of the outer shaft 
        ctx->dutyRight = ctx->speedPidController.compute(ctx->speedSetPoint, ctx->speedRightIntegral, ctx->speedRightLastError, ctx->currentSpeedRight, dt);
        ctx->motorRight.setSpeed(ctx->dutyRight);  

        ctx->webServer.update_telemetry(ctx->pitch, ctx->speedSetPoint, ctx->currentSpeedLeft, rmseSpeedErrorLeft);

        if (ctx->logCounter > 20) {
         ESP_LOGI(TAG, "Pitch: %.2f, Speed Set Point: %.2f, Encoder Speed Left: %.2f, dt: %.8f, RMSE: %.3f",
            ctx->pitch, ctx->speedSetPoint, ctx->currentSpeedRight, dt, rmseSpeedErrorLeft);  
            ctx->logCounter = 0;         
        }
        //ctx->logCounter++;
}
