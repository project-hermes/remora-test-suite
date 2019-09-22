#include <Arduino.h>
#include <Wire.h>

#include <Dive.hpp>
#include <hal/TSYS01.hpp>
#include <hal/remora-hal.h>

void setup(){
    Serial.begin(115200);
    pinMode(GPIO_SENSOR_POWER, OUTPUT);
    digitalWrite(GPIO_SENSOR_POWER, LOW);
    delay(10);
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(10);

    Dive d;
    d.Start(432432432l, 233.2322,432.2323);
    tsys01 temperatureSensor = tsys01();
    Record tempRecord = Record{temperatureSensor.getTemp(), 2};
    d.NewRecord(tempRecord);
}

void loop(){
}