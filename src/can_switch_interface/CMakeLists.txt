cmake_minimum_required(VERSION 3.5)
project(can_switch_interface)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(can_switch_interface SHARED
  src/can_switch_interface.cpp
)

rclcpp_components_register_node(can_switch_interface
  PLUGIN "CanSwitchInterface::CanSwitchInterface"
  EXECUTABLE can_switch_interface_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
