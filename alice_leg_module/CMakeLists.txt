################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(alice_leg_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  cmake_modules
  robotis_math
  robotis_framework_common
  robotis_controller_msgs
  heroehs_math
  alice_msgs
  alice_balance_control
)

find_package(Eigen REQUIRED)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES alice_leg_module
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIRS}
)

add_library(alice_leg_module
  src/alice_leg_module.cpp
  src/ros_communication.cpp
)

add_dependencies(alice_leg_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(alice_leg_module ${yaml_cpp_LIBRARIES} ${catkin_LIBRARIES})

