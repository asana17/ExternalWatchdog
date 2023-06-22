cmake_minimum_required(VERSION 3.14)
project(emergency_stop_operator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(emergency_stop_operator SHARED
  src/emergency_stop_operator/emergency_stop_operator.cpp
)

rclcpp_components_register_node(emergency_stop_operator
  PLUGIN "emergency_stop_operator::EmergencyStopOperator"
  EXECUTABLE emergency_stop_operator_node)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
