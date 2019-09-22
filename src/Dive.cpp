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
    diveRecords = new Record[600];
    return ID;
}

int Dive::NewRecord()
{
    diveRecords[currentRecords] = Record{Temp:2, Depth:1};
    currentRecords++;
    if (currentRecords == siloSize)
    {
        writeSilo();
        delete[] diveRecords;
    }
    return 0;
}