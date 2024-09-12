#ifndef DIFFDRIVE_SERIAL_WHEEL_H
#define DIFFDRIVE_SERIAL_WHEEL_H

#include <string>

class Wheel
{
public:
    std::string name = "";
    double cmdTicksPerSecond = 0;
    long int encoderPos = 0;
    double rotRadians = 0;
    double velRadiansPerSecond = 0;
    double ticksPerRadian = 0;

    Wheel() = default;

    Wheel(const std::string &wheelName, int ticksPerRevolution);

    void setup(const std::string &wheelName, int ticksPerRevolution);

    double calcEncRadians();
};

#endif // DIFFDRIVE_SERIAL_WHEEL_H