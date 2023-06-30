cmake_minimum_required(VERSION 3.14)
project(supervisor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(supervisor_node SHARED
  src/supervisor.cpp
  src/can_switch_interface.cpp
  src/voter.cpp
)

rclcpp_components_register_node(supervisor_node
  PLUGIN "supervisor::Supervisor"
  EXECUTABLE supervisor
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
