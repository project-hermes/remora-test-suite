#include <Arduino.h>
#include <freertos/task.h>

void ReadSensors(void *parms);
void WriteSpiffs(void *parms);

QueueHandle_t spiffsQueue;

struct SensorReading
{
    double temp1;
    double temp2;
    double depth;
    double time;
};

void setup()
{
    Serial.begin(115200);
    delay(1000);

    spiffsQueue = xQueueCreate(10, sizeof(struct SensorReading *));
    if (spiffsQueue == 0)
    {
        Serial.printf("Failed to create spiffs queue!");
        while (true)
        {
        }
    }

    xTaskCreate(ReadSensors, "Read Sensors", 10000, NULL, 1, NULL);
    xTaskCreate(WriteSpiffs, "Write Spiffs", 10000, NULL, 2, NULL);
}

void loop() {}

void ReadSensors(void *parms)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        Serial.print("Reading sensors... ");

        struct SensorReading *queueReading;
        struct SensorReading reading;
        queueReading = &reading;

        reading.time = millis();
        reading.depth = 12.0;
        reading.temp1 = 19.43;
        reading.temp2 = 18.93;

        Serial.printf("%lf %lf %lf %lf\n", reading.time, reading.depth, reading.temp1, reading.temp2);

        xQueueSend(spiffsQueue, (void *)&queueReading, (TickType_t)5);

        vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    }
}

void WriteSpiffs(void *parms)
{
    struct SensorReading *queueReading;
    while (true)
    {
        if (spiffsQueue != 0)
        {
            if(xQueueReceive(spiffsQueue, &(queueReading), (TickType_t)5)){
                Serial.print("Saving dive... ");
                Serial.printf(" %lf %lf %lf %lf\n", queueReading->time, queueReading->depth, queueReading->temp1, queueReading->temp2);
            }
        }
    }
}