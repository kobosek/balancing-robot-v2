#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

QueueHandle_t motorVelocityQueue;
QueueHandle_t pitchQueue;

TaskHandle_t motorVelocityTaskHandle;
TaskHandle_t pitchTaskHandle;
TaskHandle_t controlTask;

esp_timer_handle_t controlLoopTimer;

void encoder_Task(void* pvParams) {
    MotorVelocityData velocity;
    while (true) {
        velocity.velocity = 0.0f;
        xQueueSend(motorVelocityQueue, &velocity, portMAX_DELAY);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sensor_task(void* pvParams) {
    PitchData pitch;
    while (true) {
        pitch.pitch = 0.0f;
        xQueueSend(pitchQueue, &pitch, portMAX_DELAY);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Hello from task and I'm running on core %d\n", xPortGetCoreID());
    }
}

void control_task(void* pvParams) {
    MotorVelocityData velocity;
    PitchData pitch;

    while (true) {
        if(xQueueReceive(pitchQueue, &pitch, portMAX_DELAY)) {
            
        }
        if(xQueueReceive(motorVelocityQueue, &velocity, portMAX_DELAY)) {

        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Hello from task and I'm running on core %d\n", xPortGetCoreID());
    }
}

struct PitchData {
    float pitch;
};
struct MotorVelocityData {
    float velocity;
}

void app_main(void) {
    pitchQueue = xQueueCreate(10, sizeof(MotorVelocityData));
    motorVelocityQueue = xQueueCreate(10, sizeof(PitchData));
    
    xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        4096,
        NULL, 
        3, 
        &pitchTaskHandle, 
        1);

    xTaskCreatePinnedToCore(
        encoder_Task,
        "encoder_Task",
        4096,
        NULL, 
        4, 
        &motorVelocityTaskHandle, 
        0);


      xTaskCreatePinnedToCore(
        control_task,
        "control_task",
        4096,
        NULL, 
        5, 
        &controlTask, 
        0);  
}