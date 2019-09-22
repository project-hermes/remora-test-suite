#include <Dive.hpp>
#include <Storage.hpp>

Dive::Dive(void){}

Dive::Dive(Storage *s){
    storage = s;
}

String Dive::Start(long time, double lat, double lng){
    ID = createID(time);
    return ID;
}