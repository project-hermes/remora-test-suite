#include <Arduino.h>

void setup()
{
    Serial.begin(115200);

    pinMode(34, INPUT);
    pinMode(35, INPUT);

    pinMode(12, OUTPUT);

    digitalWrite(12, HIGH);
}

void loop()
{
    delay(1000);
    Serial.printf("bat %d\n", analogRead(34));
    Serial.printf("vcc %d\n", digitalRead(35));
    if(digitalRead(35)==1){
        digitalWrite(12, HIGH);
    }else{
        digitalWrite(12, LOW);
    }
}