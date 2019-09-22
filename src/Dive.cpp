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
    diveRecords = new Record[siloRecordSize];
    return ID;
}

int Dive::NewRecord(Record r)
{
    diveRecords[currentRecords] = r;
    currentRecords++;
    if (currentRecords == siloRecordSize)
    {
        writeSilo();
        delete[] diveRecords;
        diveRecords = new Record[siloRecordSize];
        currentRecords = 0;
    }
    return 0;
}