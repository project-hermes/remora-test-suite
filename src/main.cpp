#include <Arduino.h>
#include "remora-hal.h"
#include <AutoConnect.h>
#include "GPS.h"

#define GPSSerial Serial2

void initGPS();

WebServer Server;
AutoConnect Portal(Server);

TinyGPSPlus gps;

void startPortal()
{
    Serial.printf("starting config portal\n");
    AutoConnectConfig acConfig("Remora", "Dive");
    acConfig.autoReconnect = false;
    acConfig.autoReset = true;
    Portal.config(acConfig);
    Portal.begin();
    while (WiFi.getMode() == WIFI_AP_STA)
    {
        Portal.handleClient();
    }
}

void displayInfo()
{
    Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}

void dive()
{
    initGPS();
    while (GPSSerial.available() > 0)
    {
        if (gps.encode(GPSSerial.read()))
        {
            displayInfo();
        }
        delay(100);
    }
}

void wakeup()
{
    uint64_t wakeup_reason = esp_sleep_get_ext1_wakeup_status();
    uint64_t mask = 1;
    int i = 0;
    while (i < 64)
    {
        if (wakeup_reason & mask)
        {
            Serial.printf("Wakeup because %d\n", i);
            if (i == GPIO_CONFIG)
            {
                dive();
            }
            else if (i == GPIO_WATER)
            {
                dive();
            }
        }
        i++;
        mask = mask << 1;
    }
}

void sleep()
{
    uint64_t wakeMask = 1ULL << GPIO_CONFIG | 1ULL << GPIO_WATER;
    esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_HIGH);
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
}

void initGpio()
{
    pinMode(GPIO_SENSOR_POWER, INPUT); //keeps the pin floating

    pinMode(GPIO_GPS_POWER, INPUT); //keeps the pin floating

    pinMode(GPIO_CONFIG, INPUT);
    pinMode(GPIO_WATER, INPUT);
    pinMode(GPIO_VCC_SENSE, INPUT);

    pinMode(GPIO_LED1, OUTPUT);
    pinMode(GPIO_LED2, OUTPUT);
    pinMode(GPIO_LED3, OUTPUT);
    pinMode(GPIO_LED4, OUTPUT);
}

void initSerial()
{
    Serial.begin(115200); //serial for USB
}

void initGPS()
{
    Serial.println("Starting GPS...");
    pinMode(GPIO_SENSOR_POWER, OUTPUT);
    digitalWrite(GPIO_SENSOR_POWER, LOW);

    pinMode(GPIO_GPS_POWER, OUTPUT);
    digitalWrite(GPIO_GPS_POWER, LOW);

    GPSSerial.begin(9600);
    while (GPSSerial.available() == 0)
    {
    }
    delay(100);
}

void setup()
{
    initSerial();
    initGpio();

    wakeup();
    sleep();
}

void loop() {}