#include <Arduino.h>

#include "leds.h"

leds::leds(){
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
    pinMode(led4, OUTPUT);
}

void leds::blink(led_t led, int freq){
    digitalWrite(led, HIGH);
    delay(freq * 1000);
    digitalWrite(led, LOW);
    delay(freq * 1000);
}

void leds::fade(led_t led, int freq){
    
}