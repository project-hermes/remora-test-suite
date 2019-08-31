#include <Arduino.h>

void TaskA(void *parms);
void TaskB(void *parms);

void setup()
{
    Serial.begin(115200);

    xTaskCreate(TaskA, "TaskA", 10000, NULL, 1, NULL);
    xTaskCreate(TaskB, "TaskB", 10000, NULL, 2, NULL);
}

void loop() {}

void TaskA(void *parms)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        Serial.print("Hello from A ");
        Serial.println(millis()/1000);
        vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    }
}

void TaskB(void *parms)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        Serial.print("Hello from B ");
        Serial.println(millis()/1000);
        vTaskDelayUntil(&xLastWakeTime, 2000 / portTICK_PERIOD_MS);
    }
}