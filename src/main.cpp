#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    Serial2.begin(9600);
}

void loop()
{
    while (Serial2.available())
    {
        Serial.print(char(Serial2.read()));
    }
}