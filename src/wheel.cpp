#include "ros2_control_serial_diffdrive_plugin/wheel.h"

#include <cmath>

Wheel::Wheel(const std::string &wheel_name, int ticks_per_rev)
{
  setup(wheel_name, ticks_per_rev);
}

void Wheel::setup(const std::string &wheel_name, int ticks_per_rev)
{
  name = wheel_name;
  ticks_per_radian = ticks_per_rev / (2 * M_PI);
}

double Wheel::calcEncRadians()
{
  return enc / ticks_per_radian;
}