cmake_minimum_required(VERSION 3.14)
project(supervisor_psim)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(supervisor_psim_node SHARED
  src/supervisor.cpp
  src/control_switch_interface.cpp
  src/voter.cpp
)

rclcpp_components_register_node(supervisor_psim_node
  PLUGIN "supervisor::SupervisorNode"
  EXECUTABLE supervisor_psim
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
