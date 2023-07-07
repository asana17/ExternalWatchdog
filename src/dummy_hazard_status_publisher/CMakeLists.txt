cmake_minimum_required(VERSION 3.14)
project(dummy_hazard_status_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(dummy_hazard_status_publisher_node SHARED
  src/dummy_hazard_status_publisher.cpp
)

rclcpp_components_register_node(dummy_hazard_status_publisher_node
  PLUGIN "dummy_hazard_status_publisher::DummyHazardStatusPublisher"
  EXECUTABLE dummy_hazard_status_publisher
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
