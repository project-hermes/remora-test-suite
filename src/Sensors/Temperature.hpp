#include <Sensors/Sensor.hpp>

using namespace std;

class Temperature: public Sensor{
    public:
        virtual double getTemp() = 0;

        double toFahrenheit(double temp){
            return (temp * (9 / 5)) + 32;
        }

        double toCelsius(double temp){
            return (temp - 32) * (5 / 9);
        }

        double toKelvin(double temp){
            return temp + 273.15;
        }
};