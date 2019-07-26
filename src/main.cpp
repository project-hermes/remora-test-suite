#include <Arduino.h>
#include <driver/ledc.h>

ledc_timer_config_t ledc_timer;
ledc_channel_config_t led_channel;
void setup()
{
    Serial.begin(115200);

    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
    ledc_timer.freq_hz = 5000;
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;

    ledc_timer_config(&ledc_timer);

    led_channel.channel = LEDC_CHANNEL_0;
    led_channel.duty = 0,
    led_channel.gpio_num = 12;
    led_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    led_channel.hpoint = 0;
    led_channel.timer_sel = LEDC_TIMER_0;

    ledc_channel_config(&led_channel);
    ledc_fade_func_install(0);
}

void loop()
{
    Serial.println("Hello World!");
    ledc_set_fade_with_time(led_channel.speed_mode, led_channel.channel, 10000, 3000);
    ledc_fade_start(led_channel.speed_mode, led_channel.channel, LEDC_FADE_NO_WAIT);
    delay(1000);
    ledc_set_fade_with_time(led_channel.speed_mode, led_channel.channel, 0, 3000);
    ledc_fade_start(led_channel.speed_mode, led_channel.channel, LEDC_FADE_NO_WAIT);
}