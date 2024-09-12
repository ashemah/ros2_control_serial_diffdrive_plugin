#include "ros2_control_serial_diffdrive_plugin/wheel.h"

#include <cmath>

Wheel::Wheel(const std::string &wheelName, int ticksPerRevolution)
{
  setup(wheelName, ticksPerRevolution);
}

void Wheel::setup(const std::string &wheelName, int ticksPerRevolution)
{
  name = wheelName;
  ticksPerRadian = ticksPerRevolution / (2 * M_PI);
}

double Wheel::calcEncRadians()
{
  return encoderPos / ticksPerRadian;
}