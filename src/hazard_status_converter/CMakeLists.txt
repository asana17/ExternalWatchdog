cmake_minimum_required(VERSION 3.14)
project(hazard_status_converter)

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

ament_auto_add_executable(hazard_status_converter
  src/hazard_status_converter.cpp
  src/hazard_status_converter_node.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
