#include <Sensors/Sensor.hpp>

using namespace std;

class Pressure : public Sensor{
    public:
        virtual double getPressure() = 0;
};