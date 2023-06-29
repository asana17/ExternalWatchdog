cmake_minimum_required(VERSION 3.14)
project(raw_vehicle_cmd_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(raw_vehicle_cmd_converter SHARED
  src/raw_vehicle_cmd_converter.cpp
)


rclcpp_components_register_node(raw_vehicle_cmd_converter
  PLUGIN "raw_vehicle_cmd_converter::RawVehicleCommandConverterNode"
  EXECUTABLE raw_vehicle_cmd_converter_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
