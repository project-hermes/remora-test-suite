#include <Arduino.h>
#include <freertos/task.h>
#include <SPIFFS.h>

void ReadSensors(void *parms);
void WriteSpiffs(void *parms);

QueueHandle_t spiffsQueue;

unsigned long DiveName;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

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

    DiveName = millis();

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

        //this should be done with a semaphore on the object
        portENTER_CRITICAL(&mux);
        reading.time = millis();
        reading.depth = 12.0;
        reading.temp1 = 19.43;
        reading.temp2 = 18.93;
        portEXIT_CRITICAL(&mux);

        Serial.printf("%lf %lf %lf %lf\n", reading.time, reading.depth, reading.temp1, reading.temp2);

        xQueueSend(spiffsQueue, (void *)&queueReading, (TickType_t)5);

        vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    }
}

/*
File structure
-records
--<timestamp>
---metadata.txt //data about the dives, gps start and such, time start and finish
---readings.csv //raw data from readings
*/
void WriteSpiffs(void *parms)
{
    struct SensorReading *queueReading;

    if (!SPIFFS.begin(true))
    {
        Serial.println("[PANIC] An Error has occurred while mounting SPIFFS");
        return;
    }

    while (true)
    {
        if (spiffsQueue != 0)
        {
            if (xQueueReceive(spiffsQueue, &(queueReading), (TickType_t)5))
            {
                Serial.print("Saving dive... ");
                char fileName[32];
                sprintf(fileName, "/records/%lu/readings.csv", DiveName);
                Serial.printf("Writting to %s\n", fileName);
                File file = SPIFFS.open(fileName, "a");

                if (!file)
                {
                    Serial.println("[ERROR] There was an error opening the file for writing");
                    break;
                }

                if (file.printf("%lf,%lf,%lf,%lf\n", queueReading->time, queueReading->depth, queueReading->temp1, queueReading->temp2))
                {
                    Serial.printf("Wrote:%lf,%lf,%lf,%lf\n", queueReading->time, queueReading->depth, queueReading->temp1, queueReading->temp2);
                }
                else
                {
                    Serial.println("[ERROR] Could not write to file");
                }

                file.close();
            }
        }
    }
}