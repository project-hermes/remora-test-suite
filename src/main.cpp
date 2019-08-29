#include <Arduino.h>

void wakeup()
{
    uint64_t wakeup_reason = esp_sleep_get_ext1_wakeup_status();
    uint64_t mask = 1;
    int i = 0;
    digitalWrite(12, HIGH);
    while (i < 64)
    {
        if (wakeup_reason & mask){
            Serial.printf("Wakeup because %d\n", i);
            digitalWrite(13, HIGH);
        }
        i++;
        mask = mask << 1;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println();

    pinMode(36, INPUT);
    pinMode(35, INPUT);

    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(15, OUTPUT);

    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    digitalWrite(14, LOW);
    digitalWrite(15, LOW);

    Serial.printf("system starting...\n");
    delay(100);

    wakeup();
    uint64_t wakeMask = 1ULL << 36;
    esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_HIGH);
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
}

void loop()
{

}