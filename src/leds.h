#ifndef leds_h
#define leds_h

class leds{
    public:
        enum led_t
        {
            led1 = 12,
            led2,
            led3,
            led4,
        } led;

        leds();
        void blink(led_t led, int freq = 1); //time is in Hz
        void fade(led_t led, int freq = 1); //seed is in Hz
};

#endif