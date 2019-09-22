#ifndef GNSS_HPP
#define GNSS_HPP

#include <Arduino.h>

#include <Navigation/Navigation.hpp>
#include <Hal/remora-hal.h>

class GNSS : public Navigation
{
public:
    HardwareSerial GPSSerial = Serial2;

    GNSS()
    {
        pinMode(GPIO_GPS_POWER, OUTPUT);
        digitalWrite(GPIO_GPS_POWER, LOW);
        GPSSerial.begin(9600);
        delay(5000); //TODO this needs to be more dynamic
    }

private:
    
};

#endif