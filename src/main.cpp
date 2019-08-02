#include <Arduino.h>
#include <driver/ledc.h>

ledc_timer_config_t ledc_timer;
ledc_channel_config_t led_channel;
void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println("Hello World!");
    delay(5);
}