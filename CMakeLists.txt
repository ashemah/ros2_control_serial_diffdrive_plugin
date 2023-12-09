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

pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)


install(
  TARGETS ros2_control_serial_diffdrive_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY controllers launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  ros2_control_serial_diffdrive_plugin
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

ament_package()


