cmake_minimum_required(VERSION 3.8)
project(supervisor_vehicle_cmd_gate)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(supervisor_vehicle_cmd_gate SHARED
  src/supervisor_vehicle_cmd_gate.cpp
)

rclcpp_components_register_node(supervisor_vehicle_cmd_gate
  PLUGIN "Supervisor_vehicle_cmd_gate::Supervisor_vehicle_cmd_gate"
  EXECUTABLE supervisor_vehicle_cmd_gate_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
