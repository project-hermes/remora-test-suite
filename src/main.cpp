#include <Arduino.h>

#include <Dive.hpp>

void setup(){
    Serial.begin(115200);
    delay(1000);
    Dive d;
    d.Start(432432432l, 233.2322,432.2323);
    Record tempRecord = Record{1, 2};
    d.NewRecord(tempRecord);
}

void loop(){
}