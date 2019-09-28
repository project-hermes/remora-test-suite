#include <Arduino.h>

#include <Wake.hpp>

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("I did an update!");

    wake();
    sleep();
}

void loop()
{
}