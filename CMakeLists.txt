cmake_minimum_required(VERSION 3.5)
project(ros2_control_serial_diffdrive_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)






add_library(ros2_control_serial_diffdrive_plugin SHARED src/ros2_control_serial_diffdrive_plugin.cpp src/wheel.cpp src/serial_comms.cpp)

target_include_directories(
  ros2_control_serial_diffdrive_plugin
  PRIVATE
  include
)
ament_target_dependencies(
  ros2_control_serial_diffdrive_plugin
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)



add_library(fake_robot SHARED src/fake_robot.cpp src/wheel.cpp)
set_property(TARGET fake_robot PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(
  fake_robot
  PRIVATE
  include
)
ament_target_dependencies(
  fake_robot
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface fake_robot_hardware.xml)




install(
  TARGETS ros2_control_serial_diffdrive_plugin fake_robot
  DESTINATION lib
)




install(
  DIRECTORY controllers launch/
  DESTINATION share/${PROJECT_NAME}
)


ament_export_libraries(
  ros2_control_serial_diffdrive_plugin
  fake_robot
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

ament_package()


