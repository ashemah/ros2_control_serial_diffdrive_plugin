#include "ros2_control_serial_diffdrive_plugin/ros2_control_serial_diffdrive_plugin.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

// #define ENABLE_LOGGING

namespace diffdrive_serial
{
  hardware_interface::CallbackReturn DiffDriveSerial::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "Configuring...");

    lastMoveTime_ = std::chrono::system_clock::now();
    lastLogTime_ = std::chrono::system_clock::now();

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    // Set up the wheels
    l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSerial"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSerial"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSerial"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSerial"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSerial"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "Finished Configuration");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffDriveSerial::export_state_interfaces()
  {
    // We need to set up a position and a velocity interface for each wheel

    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.rotRadians));
    state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.velRadiansPerSecond));
    state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.rotRadians));
    state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.velRadiansPerSecond));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffDriveSerial::export_command_interfaces()
  {
    // We need to set up a velocity command interface for each wheel

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmdTicksPerSecond));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmdTicksPerSecond));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffDriveSerial::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "Re-Configuring...");
    // Set up the Arduino
    serial_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveSerial::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "Cleaning up ...please wait...");
    // if (serial_.connected())
    // {
    //   serial_.disconnect();
    // }
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveSerial::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "Starting Controller...");

    serial_.activateMotors();
    motorsAreActive_ = true;

    serial_.setPidValues(30, 20, 0, 100);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveSerial::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "Stopping Controller...");
    serial_.deactivateMotors();
    motorsAreActive_ = false;

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  bool DiffDriveSerial::cmpf(float A, float B, float epsilon)
  {
    return (fabs(A - B) < epsilon);
  }

  void DiffDriveSerial::throttledLogState() {

  #ifdef ENABLE_LOGGING
      auto nowTime = std::chrono::system_clock::now();

      std::chrono::duration<double> diff = (nowTime - lastLogTime_);

      if (diff.count() > 1) {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "LEFT C %f E %ld P %f V %f, RIGHT C %f E %ld P %f V %f", l_wheel_.cmdTicksPerSecond, l_wheel_.encoderPos, l_wheel_.rotRadians, l_wheel_.velRadiansPerSecond, r_wheel_.cmdTicksPerSecond, r_wheel_.encoderPos, r_wheel_.rotRadians, r_wheel_.velRadiansPerSecond);
        lastLogTime_ = nowTime;
      }
#endif

  }

  hardware_interface::return_type DiffDriveSerial::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    // TODO fix chrono duration

    if (!serial_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    if (motorsAreActive_) {
      serial_.readEncoderValues(l_wheel_.encoderPos, r_wheel_.encoderPos);

      double deltaSeconds = period.seconds();

      double pos_prev = l_wheel_.rotRadians;
      l_wheel_.rotRadians = l_wheel_.calcEncRadians();
      l_wheel_.velRadiansPerSecond = (l_wheel_.rotRadians - pos_prev) / deltaSeconds;

      pos_prev = r_wheel_.rotRadians;
      r_wheel_.rotRadians = r_wheel_.calcEncRadians();
      r_wheel_.velRadiansPerSecond = (r_wheel_.rotRadians - pos_prev) / deltaSeconds;
    }
    else {
      l_wheel_.velRadiansPerSecond = 0;
      r_wheel_.velRadiansPerSecond = 0;
    }

    throttledLogState();
    
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type DiffDriveSerial::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    if (!serial_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    bool isStopped = cmpf(l_wheel_.cmdTicksPerSecond, 0) || cmpf(r_wheel_.cmdTicksPerSecond, 0);

    // Calculate if we should deactivate the motors
    if (!isStopped)
    {
      // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "IS MOVING");

      if (!motorsAreActive_)
      {
        // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "ACTIVATE");
        serial_.activateMotors();
        motorsAreActive_ = true;
      }

      auto nowTime = std::chrono::system_clock::now();
      lastMoveTime_ = nowTime;
    }
    else
    {
      // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "<NOT MOVING>");

      auto nowTime = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = nowTime - lastMoveTime_;
      double deltaSeconds = diff.count();

      if (deltaSeconds > 5 && motorsAreActive_)
      {
        // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSerial"), "DISABLING MOTORS");
        serial_.deactivateMotors();
        motorsAreActive_ = false;
      }
    }

    serial_.setMotorValues(l_wheel_.cmdTicksPerSecond * l_wheel_.ticksPerRadian, r_wheel_.cmdTicksPerSecond * r_wheel_.ticksPerRadian);

    throttledLogState();

    return hardware_interface::return_type::OK;
  }
} // namespace diffdrive_serial

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    diffdrive_serial::DiffDriveSerial,
    hardware_interface::SystemInterface)
