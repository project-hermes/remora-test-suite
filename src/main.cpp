#include <Arduino.h>

#include <Wake.hpp>

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("version 2");

    wake();
    sleep();
}

void loop()
{
}