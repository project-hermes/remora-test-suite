#include <Arduino.h>
#include "tsys01.h"
#include "ms5837.h"
#include <SPIFFS.h>
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

void createDir(fs::FS &fs, const char *path)
{
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path))
    {
        Serial.println("Dir created");
    }
    else
    {
        Serial.println("mkdir failed");
    }
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("Message appended");
    }
    else
    {
        Serial.println("Append failed");
    }
    file.close();
}

void saveDive()
{
    File file2 = SPIFFS.open("/dive.txt");

    if (!file2)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.println("saving file to sd card...");

    while (file2.available())
    {
        writeFile(SD, "/dive.txt", file2.readString().c_str());
    }

    file2.close();
}

void dive()
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    File file = SPIFFS.open("/dive.txt", FILE_WRITE);
    if (!file)
    {
        Serial.println("There was an error opening the file for writing");
        return;
    }

    tsys01 *tempSensor;
    tempSensor = new tsys01();
    ms5837 *depthSensor;
    depthSensor = new ms5837();
    Serial.println("Creating dive...");

    do
    {
        double temp = tempSensor->readTemp();
        float depth = depthSensor->readDepth();
        float pressure = depthSensor->readPressure();
        if (file.printf("%f,%f,%f\n", depth, pressure, temp))
        {
            Serial.printf("Wrote:%f %f %f\n", depth, pressure, temp);
            digitalWrite(13, HIGH);
            delay(500);
            digitalWrite(13, LOW);
            delay(500);
        }
        else
        {
            Serial.println("File write failed");
            delay(1000);
        }
    } while (digitalRead(36));
    Serial.println("Saving...");
    file.close();
    Serial.println("Saved!");

    saveDive();
}

void wakeup()
{
    uint64_t wakeup_reason = esp_sleep_get_ext1_wakeup_status();
    uint64_t mask = 1;
    int i = 0;
    digitalWrite(12, HIGH);
    while (i < 64)
    {
        if (wakeup_reason & mask)
        {
            Serial.printf("Wakeup because %d\n", i);
            digitalWrite(13, HIGH);
            dive();
        }
        i++;
        mask = mask << 1;
    }
}

void initSD()
{
    if (!SD.begin(5))
    {
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
        Serial.println("MMC");
    }
    else if (cardType == CARD_SD)
    {
        Serial.println("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
        Serial.println("SDHC");
    }
    else
    {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void setup()
{
    Serial.begin(115200);

    pinMode(36, INPUT);
    pinMode(39, INPUT);

    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(15, OUTPUT);

    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    digitalWrite(14, LOW);
    digitalWrite(15, LOW);

    pinMode(32, OUTPUT);
    digitalWrite(32, LOW);
    delay(100);

    Wire.begin(22, 21);

    SPI.begin(18, 19, 23);

    initSD();

    wakeup();
    uint64_t wakeMask = 1ULL << 39 | 1ULL << 36;
    esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_HIGH);
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
}

void loop()
{
}