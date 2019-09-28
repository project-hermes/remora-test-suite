#include <Arduino.h>
#include <Wire.h>
#include <TimeLib.h>
#include <AutoConnect.h>

#include <Dive.hpp>
#include <hal/TSYS01.hpp>
#include <hal/MS5837.hpp>
#include <hal/remora-hal.h>
#include <Storage/SecureDigital.hpp>
#include <Navigation/GNSS.hpp>
#include <Utils.hpp>
#include <Wake.hpp>

void wake()
{
    uint64_t wakeup_reason = esp_sleep_get_ext1_wakeup_status();
    uint64_t mask = 1;
    int i = 0;
    while (i < 64)
    {
        if (wakeup_reason & mask)
        {
            Serial.printf("Wakeup because %d\n", i);
            if (i == GPIO_WATER) //dive
            {
                pinMode(GPIO_SENSOR_POWER, OUTPUT);
                digitalWrite(GPIO_SENSOR_POWER, LOW);
                delay(10);
                Wire.begin(I2C_SDA, I2C_SCL);
                delay(10);

                GNSS gps = GNSS();
                SecureDigital sd = SecureDigital();
                Dive d(&sd);
                tsys01 temperatureSensor = tsys01();
                ms5837 depthSensor = ms5837();

                if (d.Start(now(), gps.getLat(), gps.getLng()) == "")
                {
                    Serial.println("error starting the dive");
                }
                else
                {
                    while (digitalRead(GPIO_VCC_SENSE) == 1)
                    {
                        Record tempRecord = Record{temperatureSensor.getTemp(), depthSensor.getDepth()};
                        d.NewRecord(tempRecord);
                    }
                    if (d.End(now(), gps.getLat(), gps.getLng()) == "")
                    {
                        Serial.println("error ending the dive");
                    }
                }

                Serial.println("done");
            }
            else if (i == GPIO_VCC_SENSE) // wifi config
            {
                startPortal();
            }
        }

        i++;
        mask = mask << 1;
    }
}

void sleep()
{
    uint64_t wakeMask = 1ULL << GPIO_VCC_SENSE | 1ULL << GPIO_WATER;
    esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_HIGH);
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
}

void startPortal()
{
    WebServer Server;
    AutoConnect Portal(Server);

    Serial.printf("starting config portal...\n");
    AutoConnectConfig acConfig("Remora Config", "cousteau");
    acConfig.autoReconnect = true;
    acConfig.autoReset = false;
    acConfig.title = "Remora Config";
    acConfig.ticker = true;
    acConfig.tickerPort = GPIO_LED1;
    acConfig.tickerOn = HIGH;
    Portal.config(acConfig);
    Portal.begin();
    while (WiFi.getMode() == WIFI_AP_STA)
    {
        Portal.handleClient();
    }
    ota();
}

void ota()
{
    String url = "https://us-central1-project-hermes-staging.cloudfunctions.net/ota";
    String file;
    HTTPClient http;

    http.begin(url);
    if (http.GET() == 200)
    {
        file = http.getString();
    }
    else
    {
        Serial.println("could not contact cloud function");
        http.end();
        return;
    }
    http.end();

    http.begin(file);
    size_t written = 0;
    size_t gotten = 1;
    if (http.GET() == 200)
    {
        gotten = http.getSize();
        Serial.println("atempting to update...");
        written = Update.writeStream(http.getStream());
    }
    http.end();

    if (written == gotten)
    {
        Serial.println("what was gotten has been written");
    }

    if (Update.end())
    {
        Serial.println("OTA done!");
        if (Update.isFinished())
        {
            Serial.println("Update successfully completed. Rebooting.");
            ESP.restart();
        }
        else
        {
            Serial.println("Update not finished? Something went wrong!");
        }
    }
    else
    {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
    }
}