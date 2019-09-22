#include <Dive.hpp>
#include <Storage.hpp>

Dive::Dive(void) {}

Dive::Dive(Storage *s)
{
    storage = s;
}

String Dive::Start(long time, double lat, double lng)
{
    ID = createID(time);
    diveRecords = new Record[siloSize];
    return ID;
}

int Dive::NewRecord(Record r)
{
    diveRecords[currentRecords] = r;
    currentRecords++;
    if (currentRecords == siloSize)
    {
        writeSilo();
        delete[] diveRecords;
        diveRecords = new Record[siloSize];
        currentRecords = 0;
    }
    return 0;
}