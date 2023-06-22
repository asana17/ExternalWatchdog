cmake_minimum_required(VERSION 3.14)
project(voter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(voter SHARED
  src/voter.cpp
)

rclcpp_components_register_node(voter
  PLUGIN "Voter::Voter"
  EXECUTABLE voter_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
