################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(alice_online_walking_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  robotis_framework_common
  robotis_controller_msgs
  heroehs_online_walking_pattern_generator
  heroehs_pd_balance_controller
  alice_kinematics_dynamics
  alice_walking_module_msgs
heroehs_math
)

find_package(Eigen REQUIRED)


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
  LIBRARIES alice_online_walking_module
  CATKIN_DEPENDS roscpp heroehs_online_walking_pattern_generator heroehs_pd_balance_controller alice_kinematics_dynamics
  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(alice_online_walking_module
  src/alice_online_walking.cpp
  src/online_walking_module.cpp
)

add_dependencies(alice_online_walking_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(alice_online_walking_module
  ${catkin_INCLUDE_DIRS}
)
