#include <mbedtls/md.h>
#include <WiFi.h>

#include <Storage.hpp>

using namespace std;

struct DiveMetadata
{
    String ID;
    long startTime;
    long endTime;
    int freq;
    int numSilos;
    int siloSize;
    float startLat;
    float startLng;
    float endLat;
    float endLng;
};

struct Record
{
    double Depth;
    double Temp;
};

struct Silo
{
    String ID;
    int order;
    Record diveRecords[];
};

class Dive
{
public:
    Dive();
    Dive(Storage *s);
    String Start(long time, double lat, double lng);
    String End(long time, double lat, double lng);
    int newRecord(Record r);

private:
    Storage *storage;
    String ID;
    DiveMetadata metadata;
    Silo silo;

    int writeSilo();
    int writeMetadata();

    String createID(long time)
    {
        byte shaResult[32];
        WiFi.mode(WIFI_MODE_STA);
        String unhashed_id = String() + WiFi.macAddress();
        Serial.println(unhashed_id);
        mbedtls_md_context_t ctx;
        mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

        const size_t payloadLength = strlen(unhashed_id.c_str());

        mbedtls_md_init(&ctx);
        mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
        mbedtls_md_starts(&ctx);
        mbedtls_md_update(&ctx, (const unsigned char *)unhashed_id.c_str(), payloadLength);
        mbedtls_md_finish(&ctx, shaResult);
        mbedtls_md_free(&ctx);

        String hash;
        for (int i = 0; i < sizeof(shaResult); i++)
        {
            char str[3];

            sprintf(str, "%02x", (int)shaResult[i]);
            hash = hash + str;
        }

        return hash;
    }
};