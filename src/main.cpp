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

#define GPSSerial Serial2

void initGPS();
void initIMU();
void initSensors();

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
//
// END HERMES DATA FORMATS
//

void sprintSample(char *buffer, DIVE_SAMPLE sample)
{
    sprintf(buffer, "%d,%d,%d, %.3f,%.3f, %d,%d, %d, %d,%d,%d, %d,%d,%d, %d,%d,%d",
            sample.tempTs, sample.tempMs, sample.presMs,
            sample.lat, sample.lng,
            sample.date, sample.time,
            sample.tempImu,
            sample.gx, sample.gy, sample.gz,
            sample.ax, sample.ay, sample.az,
            sample.mx, sample.my, sample.mz);
}

void printSample(DIVE_SAMPLE sample)
{
    char buffer[512];
    sprintSample(buffer, sample);
    Serial.println(buffer);
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
        sprintf(buffer, "%.6f,%.6f", gpsReader.location.lat(), gpsReader.location.lng());
    }
    else
    {
        sprintf(buffer, "INVALID");
    }

    sprintf(buffer, "  Date/Time: ");
    if (gpsReader.date.isValid())
    {
        sprintf(buffer, "%d-%02d-%02d", gpsReader.date.year(), gpsReader.date.month(), gpsReader.date.day());
    }
    else
    {
        sprintf(buffer, "INVALID");
    }

    sprintf(buffer, " ");
    if (gpsReader.time.isValid())
    {
        sprintf(buffer, "%02d:%02d:%02d.%02d", gpsReader.time.hour(), gpsReader.time.minute(), gpsReader.time.second(), gpsReader.time.centisecond());
    }
    else
    {
        sprintf(buffer, "INVALID");
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
}

void diveEnd(DIVE_SAMPLE *diveData, const char *message)
{
    Serial.printf("Ending dive: %s\n", message);
    // TODO: write to file!
}

const int START_WET = 3;
const int END_DRY = 5;
const int MAX_SAMPLES = 10;

// returns sample count
int dive()
{
    int consecWet = 0;
    int consecDry = 0;
    int thisWater;
    int lastWater = 1; // last water sensor read
    int flipCount = 0; // how many times did we change wet/dry state?
    int samples = 0;
    bool diving = false;

    /*
    timer = timerBegin(1,1000000,true);
    timerAttachInterrupt(timer, diveInner, false);
    timerReadSeconds(timer);
    */

    diveStart();

    tsys01 *tempSensor = new tsys01();
    ms5837 *depthSensor = new ms5837();

    DIVE_SAMPLE diveData[MAX_SAMPLES];

    int blink = HIGH;
    int waitInterval = 100;

    while (true) // panic-to-reset?!
    //for (int i = 0; i < MAX_SAMPLES * 2; i++)
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
            consecDry = 0;
        }
        else
        {
            consecDry++;
            consecWet = 0;
        }
        Serial.printf("Water: %d\n", thisWater);

        // require 15 seconds wet to start dive sampling
        if (!diving && consecWet > START_WET)
        {
            diving = true;
            digitalWrite(GPIO_LED3, HIGH); // dive indicator
        }

        // While diving, sample every second
        if (diving)
        {
            digitalWrite(GPIO_LED4, blink); // heartbeat
            blink = 1 - blink;              // heartbeat
            diveData[samples] = diveSample(tempSensor, depthSensor);
            samples++;
            // HERE: trips "Stack canary watchpoint triggered (loopTask)"
            printSample(diveData[samples]);
        }

        // end dive at 30 seconds dry, or at full memory
        if (consecDry >= END_DRY || samples >= MAX_SAMPLES)
        {
            digitalWrite(GPIO_LED3, LOW); // dive indicator
            diving = false;
            break;
        }

        // TODO: delay until next second; semi-busy wait (or use timer-and-interrupt correctly)
        delay(1000);
    }

    char buffer[255];
    sprintf(buffer, "Stats: S: %d, FC: %d", samples, flipCount);
    diveEnd(diveData, buffer);
    return samples;
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
            if (i == GPIO_WATER)
            {
                diveSamples = 1;
            }
            else if (i == GPIO_CONFIG)
            {
                diveSamples = 5;
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

void setup()
{
    initSerial();
    Serial.println("Setup:");
    initGpio();

    digitalWrite(GPIO_LED1, HIGH); // running indicator

    wakeup();
    sleep();
}

void loop()
{
    Serial.println("Loop:");
}
