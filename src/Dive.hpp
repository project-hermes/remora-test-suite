#include <string>
#include <Storage.hpp>

using namespace std;

struct DiveMetadata{
    string ID;
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

struct Silo{
    string ID;
    int order;
    Record diveRecords[600];
};

struct Record{
    double Depth;
    double Temp;
};

class Dive
{
public:
    Dive(Storage *s);
    string Start(long time, double lat, double lng);
    string End(long time, double lat, double lng);
    int newRecord(Record r);

private:
    Storage *storage;
    string ID;
    DiveMetadata metadata;
    Silo silo;

    int writeSilo();
    int writeMetadata();

    string createID(long time, ){

    }
};