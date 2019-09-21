#include <Sensors/Sensor.hpp>

using namespace std;

class Depth:public Sensor{
    public:
        virtual double getDepth() = 0;
};