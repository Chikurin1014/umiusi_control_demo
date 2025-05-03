cmake_minimum_required(VERSION 3.8)
project(umiusi_control_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_hardware SHARED
  src/${PROJECT_NAME}/hardware/thruster_joint.cpp
  src/${PROJECT_NAME}/hardware/thruster_thrust.cpp
  include/${PROJECT_NAME}/hardware/thruster_joint.hpp
  include/${PROJECT_NAME}/hardware/thruster_thrust.hpp
)
ament_auto_add_library(${PROJECT_NAME}_controller SHARED
  src/${PROJECT_NAME}/umiusi_controller.cpp
  include/${PROJECT_NAME}/umiusi_controller.hpp
)
pluginlib_export_plugin_description_file(hardware_interface plugin/umiusi_hardware_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface plugin/umiusi_controller_plugin.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  urdf
  config
  launch
)
