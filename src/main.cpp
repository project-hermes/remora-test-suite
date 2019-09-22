#include <Arduino.h>
#include <Wire.h>

#include <Dive.hpp>
#include <hal/TSYS01.hpp>
#include <hal/MS5837.hpp>
#include <hal/remora-hal.h>
#include <Storage/SecureDigital.hpp>

void setup()
{
    Serial.begin(115200);
    pinMode(GPIO_SENSOR_POWER, OUTPUT);
    digitalWrite(GPIO_SENSOR_POWER, LOW);
    delay(10);
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(10);

    SecureDigital sd = SecureDigital();
    Dive d(&sd);
    tsys01 temperatureSensor = tsys01();
    ms5837 depthSensor = ms5837();
    if (d.Start(432432432l, 233.2322, 432.2323) == "")
    {
        Serial.println("error starting the dive");
    }
    else
    {
        for (int i = 0; i < 300; i++)
        {
            Record tempRecord = Record{temperatureSensor.getTemp(), depthSensor.getDepth()};
            d.NewRecord(tempRecord);
        }
        if (d.End(23423, 234.434, 2343.543) == "")
        {
            Serial.println("error ending the dive");
        }
    }
    Serial.println("done");
}

void loop()
{
}