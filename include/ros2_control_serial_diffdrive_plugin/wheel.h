#ifndef DIFFDRIVE_SERIAL_WHEEL_H
#define DIFFDRIVE_SERIAL_WHEEL_H

#include <string>

class Wheel
{
public:
    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double ticks_per_radian = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int ticks_per_rev);

    void setup(const std::string &wheel_name, int ticks_per_rev);

    double calcEncRadians();
};

#endif // DIFFDRIVE_SERIAL_WHEEL_H