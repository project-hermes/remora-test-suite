#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    Serial2.begin(9600);

    pinMode(32, OUTPUT);
    pinMode(39, INPUT);
    digitalWrite(32, LOW);

}

void loop()
{
    while (Serial2.available())
    {
        Serial.print(char(Serial2.read()));
        if(digitalRead(39)==1){
            pinMode(32, INPUT);
            break;
        }
    }
}