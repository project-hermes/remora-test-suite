#include <Arduino.h>
#include "remora-hal.h"
#include <AutoConnect.h>
#include "GPS.h"
#include "SparkFunLSM9DS1.h"
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "tsys01.h"
#include "ms5837.h"
#include <time.h>

#define GPSSerial Serial2

void initGPS();
void initIMU();
void initSensors();
bool initSD();

WebServer Server;
AutoConnect Portal(Server);

LSM9DS1 imu;
TinyGPSPlus gpsReader;

#define LSM9DS1_M 0x1E  // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
#define PRINT_SPEED 250             // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

#define DECLINATION -9.13 // Declination (degrees) in Raleigh, NC.

#define HOLD_FOR_DUMP // No sleep until dumped to serial

//
// HERMES DATA FORMATS
//
typedef struct GPS_LOC
{
    float latitude, longitude; // -90 to 90, 180 to -180
} GPS_LOC;

// Raw depth storage
typedef struct RAW_DEPTH
{
    uint32_t data1;
    uint32_t data2;
} RAW_DEPTH;

typedef struct DIVE_SAMPLE
{
    uint32_t tempTs;
    uint32_t tempMs;
    uint32_t presMs;
    double lat, lng;
    uint32_t date, time;
    int16_t tempImu;
    int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
    int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
    int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
} DIVE_SAMPLE;

typedef struct DIVE_DATA
{
    char depth[2];
    char temp1[2];
    char temp2[2];
} DIVE_DATA;

typedef struct DIVE_INFO
{
    char diveId;
    float latStart, latEnd, longStart, longEnd;
    int dataCount;
    unsigned long timeStart, timeEnd;
    DIVE_DATA *diveData;
} DIVE_INFO;

// Data storage
DIVE_DATA lastSample;
DIVE_INFO diveInfo;
RAW_DEPTH lastDepth;

const int MAX_SAMPLES = 1200;
DIVE_SAMPLE DiveData[MAX_SAMPLES];
int CurSamples = 0;
bool Dumped = false;

int DiveCount = 0;
//
// END HERMES DATA FORMATS
//

int sprintSampleNo(char *buffer, int sampleNo)
{
    return sprintf(buffer, "%d,%d,%d, %.3f,%.3f, %d,%d, %d, %d,%d,%d, %d,%d,%d, %d,%d,%d\n",
                   DiveData[sampleNo].tempTs, DiveData[sampleNo].tempMs, DiveData[sampleNo].presMs,
                   DiveData[sampleNo].lat, DiveData[sampleNo].lng,
                   DiveData[sampleNo].date, DiveData[sampleNo].time,
                   DiveData[sampleNo].tempImu,
                   DiveData[sampleNo].gx, DiveData[sampleNo].gy, DiveData[sampleNo].gz,
                   DiveData[sampleNo].ax, DiveData[sampleNo].ay, DiveData[sampleNo].az,
                   DiveData[sampleNo].mx, DiveData[sampleNo].my, DiveData[sampleNo].mz);
}

void printSampleNo(int sampleNo)
{
    char buffer[255];
    sprintSampleNo(buffer, sampleNo);
    Serial.print(buffer);
}

void startPortal()
{
    Serial.printf("starting config portal\n");
    AutoConnectConfig acConfig("Remora", "Dive");
    acConfig.autoReconnect = false;
    acConfig.autoReset = true;
    Portal.config(acConfig);
    Portal.begin();
    while (WiFi.getMode() == WIFI_AP_STA)
    {
        Portal.handleClient();
    }
}

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

void printGyro()
{
    // Now we can use the gx, gy, and gz variables as we please.
    // Either print them as raw ADC values, or calculated in DPS.
    Serial.print("G: ");
#ifdef PRINT_CALCULATED
    // If you want to print calculated values, you can use the
    // calcGyro helper function to convert a raw ADC value to
    // DPS. Give the function the value that you want to convert.
    Serial.print(imu.calcGyro(imu.gx), 2);
    Serial.print(", ");
    Serial.print(imu.calcGyro(imu.gy), 2);
    Serial.print(", ");
    Serial.print(imu.calcGyro(imu.gz), 2);
    Serial.println(" deg/s");
#elif defined PRINT_RAW
    Serial.print(imu.gx);
    Serial.print(", ");
    Serial.print(imu.gy);
    Serial.print(", ");
    Serial.println(imu.gz);
#endif
}

void printAccel()
{
    // Now we can use the ax, ay, and az variables as we please.
    // Either print them as raw ADC values, or calculated in g's.
    Serial.print("A: ");
#ifdef PRINT_CALCULATED
    // If you want to print calculated values, you can use the
    // calcAccel helper function to convert a raw ADC value to
    // g's. Give the function the value that you want to convert.
    Serial.print(imu.calcAccel(imu.ax), 2);
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.ay), 2);
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.az), 2);
    Serial.println(" g");
#elif defined PRINT_RAW
    Serial.print(imu.ax);
    Serial.print(", ");
    Serial.print(imu.ay);
    Serial.print(", ");
    Serial.println(imu.az);
#endif
}

void printMag()
{
    // Now we can use the mx, my, and mz variables as we please.
    // Either print them as raw ADC values, or calculated in Gauss.
    Serial.print("M: ");
#ifdef PRINT_CALCULATED
    // If you want to print calculated values, you can use the
    // calcMag helper function to convert a raw ADC value to
    // Gauss. Give the function the value that you want to convert.
    Serial.print(imu.calcMag(imu.mx), 2);
    Serial.print(", ");
    Serial.print(imu.calcMag(imu.my), 2);
    Serial.print(", ");
    Serial.print(imu.calcMag(imu.mz), 2);
    Serial.println(" gauss");
#elif defined PRINT_RAW
    Serial.print(imu.mx);
    Serial.print(", ");
    Serial.print(imu.my);
    Serial.print(", ");
    Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    float heading;
    if (my == 0)
        heading = (mx < 0) ? PI : 0;
    else
        heading = atan2(mx, my);

    heading -= DECLINATION * PI / 180;

    if (heading > PI)
        heading -= (2 * PI);
    else if (heading < -PI)
        heading += (2 * PI);

    // Convert everything from radians to degrees:
    heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    Serial.print("Pitch, Roll: ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    Serial.print("Heading: ");
    Serial.println(heading, 2);
}

void sprintGPS(char *buffer)
{
    sprintf(buffer, "Location: ");
    if (gpsReader.location.isValid())
    {
        sprintf(buffer, "%s%.6f,%.6f", buffer, gpsReader.location.lat(), gpsReader.location.lng());
    }
    else
    {
        sprintf(buffer, "%sINVALID", buffer);
    }

    sprintf(buffer, "%s  Date/Time: ", buffer);
    if (gpsReader.date.isValid())
    {
        sprintf(buffer, "%s%d-%02d-%02d", buffer, gpsReader.date.year(), gpsReader.date.month(), gpsReader.date.day());
    }
    else
    {
        sprintf(buffer, "%sINVALID", buffer);
    }

    sprintf(buffer, "%s ", buffer);
    if (gpsReader.time.isValid())
    {
        sprintf(buffer, "%s%02d:%02d:%02d.%02d", buffer, gpsReader.time.hour(), gpsReader.time.minute(), gpsReader.time.second(), gpsReader.time.centisecond());
    }
    else
    {
        sprintf(buffer, "%sINVALID", buffer);
    }
}

void printGPS()
{
    char buffer[255];
    sprintGPS(buffer);
    Serial.println(buffer);
}

const int maxSentences = 20; // high; calibrate to full sample from hardware?
int readGPS()
{
    int sentenceCount = 0;
    while (sentenceCount < maxSentences && GPSSerial.available() > 0)
    {
        if (gpsReader.encode(GPSSerial.read()))
        {
            sentenceCount++;
        }
    }

    if (sentenceCount == 0)
    {
        Serial.println("GPS Err: no new NMEA sentences");
    }
    else
    {
        Serial.printf("GPS: read %d\n", sentenceCount);
        Serial.printf("GPS: c:%d f:%d P:%d F:%d\n",
                      gpsReader.charsProcessed(), gpsReader.sentencesWithFix(),
                      gpsReader.passedChecksum(), gpsReader.failedChecksum());
    }
    return sentenceCount;
}

DIVE_SAMPLE diveSample(tsys01 *tempSensor, ms5837 *depthSensor)
{
    DIVE_SAMPLE sample;

    // Core sensors first
    sample.tempTs = tempSensor->readTemp();
    sample.tempMs = depthSensor->readTemp();
    sample.presMs = depthSensor->readDepth();

    Serial.println("Core done.");

    // Update the sensor values whenever new data is available
    if (imu.gyroAvailable())
    {
        imu.readGyro();
        sample.gx = imu.gx;
        sample.gy = imu.gy;
        sample.gz = imu.gz;
    }
    else
    {
        sample.gx = 0;
        sample.gy = 0;
        sample.gz = 0;
    }
    if (imu.accelAvailable())
    {
        imu.readAccel();
        sample.ax = imu.ax;
        sample.ay = imu.ay;
        sample.az = imu.az;
    }
    else
    {
        sample.ax = 0;
        sample.ay = 0;
        sample.az = 0;
    }
    if (imu.magAvailable())
    {
        imu.readMag();
        sample.mx = imu.mx;
        sample.my = imu.my;
        sample.mz = imu.mz;
    }
    else
    {
        sample.mx = 0;
        sample.my = 0;
        sample.mz = 0;
    }
    if (imu.tempAvailable())
    {
        imu.readTemp();
        sample.tempImu = imu.temperature;
    }
    else
    {
        sample.tempImu = 0;
    }

    Serial.println("IMU done.");

    if (GPSSerial.available() > 0)
    {
        readGPS();
    }
    else
    {
        Serial.println(F("GPS Err: no new samples"));
    }
    if (gpsReader.location.isValid())
    {
        sample.lat = gpsReader.location.lat();
        sample.lng = gpsReader.location.lng();
    }
    else
    {
        sample.lat = 0;
        sample.lng = 0;
    }
    if (gpsReader.date.isValid())
    {
        sample.date = gpsReader.date.value();
    }
    else
    {
        sample.date = 0;
    }
    if (gpsReader.time.isValid())
    {
        sample.time = gpsReader.time.value();
    }
    else
    {
        sample.time = 0;
    }

    Serial.println("GPS done.");

    return sample;
}

void printSample()
{
    tsys01 *tempSensor;
    ms5837 *depthSensor;
    tempSensor = new tsys01();
    depthSensor = new ms5837();

    diveSample(tempSensor, depthSensor);

    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    printGPS();
    printAttitude(imu.ax, imu.ay, imu.az,
                  -imu.my, -imu.mx, imu.mz);
    Serial.printf("I Temp %d\n", imu.temperature);

    Serial.printf("T Temp %f\n", tempSensor->readTemp());
    Serial.printf("M Temp %f\n", depthSensor->readTemp());
    Serial.printf("Depth %f\n", depthSensor->readDepth());
}

void diveStart()
{
    initSensors();
    initGPS();
    initIMU();
    initSD();
    DiveCount++;
    Serial.printf("Starting dive %d\n", DiveCount);
    char path[32];
    sprintf(path, "logs/%d.log", DiveCount);
    writeFile(SD, path, "Dive Start!\n");
}

const int PRINT_LEN = 1024;
const int LINE_MAX = 150;
void diveEnd(int samples, const char *message)
{
    Serial.printf("Ending dive %d with %d samples: %s\n", DiveCount, samples, message);
    // TODO: write samples to file!
    char path[32];
    sprintf(path, "logs/%d.log", DiveCount);

    File file = SD.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.printf("Failed to open file %s for appending!\n", path);
        return;
    }

    char lines[PRINT_LEN];
    int lineLen;
    int sampleNo = 0;
    while (sampleNo < samples)
    {
        lineLen = 0;
        while (lineLen < PRINT_LEN - LINE_MAX && sampleNo < samples)
        {
            lineLen += sprintSampleNo(&lines[lineLen], sampleNo);
            sampleNo++;
        }
        if (!file.print(lines))
        {
            Serial.printf("Append %d failed\n", sampleNo);
        }
        //appendFile(fs, path, lines);
    }
    if (!file.print("Dive End!"))
    {
        Serial.println("Last append failed");
    }
    //appendFile(fs, path, "Dive End!\n");

    file.close();

    Serial.printf("Dive %s Written.\n", path);
}

void printCalibrations(tsys01 *tempSensor, ms5837 *depthSensor)
{
    uint16_t *calibrations;
    Serial.println("Calibrations:");
    calibrations = tempSensor->calibration();
    Serial.printf("Temp: [%d, %d, %d, %d, %d, %d, %d, %d]\n",
                  calibrations[0], calibrations[1], calibrations[2], calibrations[3],
                  calibrations[4], calibrations[5], calibrations[6], calibrations[7]);
    calibrations = depthSensor->calibration();
    Serial.printf("Pres: [%d, %d, %d, %d, %d, %d, %d, %d]\n",
                  calibrations[0], calibrations[1], calibrations[2], calibrations[3],
                  calibrations[4], calibrations[5], calibrations[6], calibrations[7]);
}

// dump saved dive to serial
int diveDump(tsys01 *tempSensor, ms5837 *depthSensor)
{
    Serial.printf("\n\nDumping Dive %d (%d) . . .\n", DiveCount, CurSamples);
    for (int i = 0; i < CurSamples; i++)
    {
        printSampleNo(i);
    }
    printCalibrations(tempSensor, depthSensor);
    Serial.printf("Dive %d (%d) Written.\n\n\n", DiveCount, CurSamples);
    Dumped = true;
}

const int START_WET = 0;    // 0 == record immediately on wake
const int END_DRY = 3600;   // >MAX_SAMPLES == always record max dive time

// returns sample count
int dive()
{
    int consecWet = 0;
    int maxWet = 0;
    int consecDry = 0;
    int thisWater;
    int lastWater = 1; // last water sensor read
    int flipCount = 0; // how many times did we change wet/dry state?
    bool diving = false;

    if (CurSamples > 0)
    {
        if (Dumped)
        {
            Serial.printf("Starting new dive; overwriting %d samples (dumped)\n", CurSamples);
            CurSamples = 0;
            Dumped = false;
        }
        else
        {
            Serial.printf("Skipping new dive; %d samples not yet dumped\n", CurSamples);
            return 0;
        }
    }
    /*
    timer = timerBegin(1,1000000,true);
    timerAttachInterrupt(timer, diveInner, false);
    timerReadSeconds(timer);
    */

    diveStart();

    tsys01 *tempSensor = new tsys01();
    ms5837 *depthSensor = new ms5837();

    int blink = HIGH;
    int waitInterval = 100;

    while (true)
    {
        // read water sensor:
        thisWater = digitalRead(GPIO_WATER);
        if (thisWater != lastWater)
        {
            flipCount++;
        }
        lastWater = thisWater;
        if (thisWater)
        {
            consecWet++;
            if (consecWet > maxWet)
            {
                maxWet = consecWet;
            }
            consecDry = 0;
        }
        else
        {
            consecDry++;
            consecWet = 0;
        }
        Serial.printf("Water: %d\n", thisWater);
        digitalWrite(GPIO_LED1, thisWater); // water indicator

        // require 15 seconds wet to start dive sampling
        if (!diving && consecWet >= START_WET)
        {
            diving = true;
            digitalWrite(GPIO_LED3, HIGH); // dive indicator
        }

        // While diving, sample every second
        if (diving)
        {
            digitalWrite(GPIO_LED4, blink); // heartbeat
            blink = 1 - blink;              // heartbeat
            DiveData[CurSamples] = diveSample(tempSensor, depthSensor);
            // HERE: trips "Stack canary watchpoint triggered (loopTask) -- [if passing dive data]"
            printSampleNo(CurSamples);
            printGPS();
            CurSamples++;
        }
        else
        {
            digitalWrite(GPIO_LED4, LOW); // heartbeat
        }

        // end dive at 30 seconds dry, or at full memory
        if (consecDry >= END_DRY || CurSamples >= MAX_SAMPLES)
        {
            digitalWrite(GPIO_LED3, LOW); // dive indicator
            diving = false;
            break;
        }

        // TODO: delay until next second; semi-busy wait (or use timer-and-interrupt correctly)
        delay(1000);
    }

    char diveDesc[255];
    sprintf(diveDesc, "Stats: S: %d, FC: %d, MW: %d", CurSamples, flipCount, maxWet);
    diveEnd(CurSamples, diveDesc);
#ifdef HOLD_FOR_DUMP
    while (!Dumped && CurSamples > 0)
    {
        Serial.println("Holding for dump . . .");
        delay(10000);
        if (digitalRead(GPIO_CONFIG))
        {
            digitalWrite(GPIO_LED4, HIGH); // heartbeat
            diveDump(tempSensor, depthSensor);
        }
    }
    Serial.printf("Dump complete; dive stats: %s\n", diveDesc);
    delay(5000);
#endif
    return CurSamples;
}

void wakeup()
{
    digitalWrite(GPIO_LED2, HIGH); // wake indicator
    Serial.println("Wakeup:");
    uint64_t wakeup_reason = esp_sleep_get_ext1_wakeup_status();
    uint64_t mask = 1;
    int i = 0;
    int diveSamples = 0;
    while (i < 64)
    {
        if (wakeup_reason & mask)
        {
            Serial.printf("Wakeup because %d\n", i);
            if (i == GPIO_CONFIG)
            {
                //diveDump();   // memory cleared on deep sleep, not useful
                diveSamples = 5;
            }
            else if (i == GPIO_WATER)
            {
                diveSamples = 1;
            }
        }
        i++;
        mask = mask << 1;
    }
    if (diveSamples > 0)
    {
        Serial.printf("Entering dive . . .\n");
        int samples = dive();
        Serial.printf("Dive done: %d samples\n", samples);
        /*
        if (!diving)
            diveStart();
        for (i = 0; i < diveSamples; i++)
        {
            Serial.printf("%d:\n", i);
            printSample();
            delay(1000);
        }
        */
    }
}

void sleep()
{
    digitalWrite(GPIO_LED2, LOW); // wake indicator
    Serial.println("Sleep:");
    uint64_t wakeMask = 1ULL << GPIO_CONFIG | 1ULL << GPIO_WATER;
    esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_HIGH);
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
}

void initGpio()
{
    pinMode(GPIO_SENSOR_POWER, INPUT); //keeps the pin floating

    pinMode(GPIO_GPS_POWER, INPUT); //keeps the pin floating

    pinMode(GPIO_CONFIG, INPUT);
    pinMode(GPIO_WATER, INPUT);
    pinMode(GPIO_VCC_SENSE, INPUT);

    pinMode(GPIO_LED1, OUTPUT);
    pinMode(GPIO_LED2, OUTPUT);
    pinMode(GPIO_LED3, OUTPUT);
    pinMode(GPIO_LED4, OUTPUT);
}

void initSerial()
{
    Serial.begin(115200); //serial for USB
}

void initSensors()
{
    pinMode(GPIO_SENSOR_POWER, OUTPUT);
    digitalWrite(GPIO_SENSOR_POWER, LOW);
    delay(100);
}

void initGPS()
{
    Serial.println("Starting GPS...");
    pinMode(GPIO_GPS_POWER, OUTPUT);
    digitalWrite(GPIO_GPS_POWER, LOW);

    GPSSerial.begin(9600);
    while (GPSSerial.available() == 0)
    {
    }
    Serial.println("GPS started, waiting for lock...");
    int gpsDelay = 5;
    delay(gpsDelay * 1000);
    Serial.printf("%d seconds passed, assuming GPS lock.\n", gpsDelay);
}

void initIMU()
{
    imu.settings.device.commInterface = IMU_MODE_I2C;
    imu.settings.device.mAddress = LSM9DS1_M;
    imu.settings.device.agAddress = LSM9DS1_AG;

    Wire.begin(I2C_SDA, I2C_SCL); // this should be done in the library

    int maxImuTries = 5;
    int imuTries = 0;
    bool imuSuccess = false;
    while (!imuSuccess && imuTries < maxImuTries)
    {
        imuSuccess = imu.begin();
        imuTries++;
        if (imuSuccess)
        {
            Serial.printf("Initiated LSM9DS1 after %d tries.\n", imuTries);
            break;
        }
        else
        {
            int imuDelay = 1;
            Serial.printf("Failed to communicate with LSM9DS1; retrying in %d seconds\n", imuDelay);
            delay(imuDelay * 1000);
        }
    }
    if (!imuSuccess)
    {
        Serial.println("Final failure to communicate with LSM9DS1");
        Serial.println("Double-check wiring.");
    }
}

bool initSD()
{
    bool success = true;
    if (!SD.begin())
    {
        Serial.println("Card Mount Failed");
        //return false;
        success = false;
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        Serial.println("No SD card attached");
        return false;
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
    return success;
}

void setup()
{
    initSerial();
    Serial.println("Setup:");
    initGpio();

    //digitalWrite(GPIO_LED1, HIGH); // running indicator

    wakeup();
    sleep();
}

void loop()
{
    Serial.println("Loop:");
}
