cmake_minimum_required(VERSION 3.8)
project(mowbot_gnss_fuser)

# Enable warnings for GCC/Clang to improve code quality.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find core ROS 2 tools for building and dependency management.
# find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

# Automatically add all dependencies from package.xml.
ament_auto_find_build_dependencies()

## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

# Build the shared library from source and header files.
ament_auto_add_library(gnss_fuser_node SHARED
  src/gnss_fuser_node.cpp
  include/mowbot/gnss_fuser/gnss_fuser_node.hpp
)

target_link_libraries(gnss_fuser_node
  Geographic
)

# Register the library as a ROS 2 component and create a standalone executable.
rclcpp_components_register_node(gnss_fuser_node
  PLUGIN "mowbot::gnss_fuser::GNSSFuser"
  EXECUTABLE gnss_fuser
)

# Package and install config/launch files to the share directory.
ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
