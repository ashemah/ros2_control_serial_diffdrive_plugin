#ifndef DIFFDRIVE_SERIAL_PLUGIN_H
#define DIFFDRIVE_SERIAL_PLUGIN_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility.h"
#include "config.h"
#include "wheel.h"
#include "serial_comms.h"

namespace diffdrive_serial
{
class DiffDriveSerial : public hardware_interface::SystemInterface
{

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveSerial);

  void throttledLogState();

  DIFFDRIVE_SERIAL_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  DIFFDRIVE_SERIAL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFFDRIVE_SERIAL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFFDRIVE_SERIAL_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_SERIAL_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  DIFFDRIVE_SERIAL_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_SERIAL_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_SERIAL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DIFFDRIVE_SERIAL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool cmpf(float A, float B, float epsilon = 0.005f);

private:
  Config cfg_;
  SerialComms serial_;

  Wheel l_wheel_;
  Wheel r_wheel_;

  std::chrono::time_point<std::chrono::system_clock> lastMoveTime_;
  std::chrono::time_point<std::chrono::system_clock> lastLogTime_;
  bool motorsAreActive_ = false;
};
}

#endif