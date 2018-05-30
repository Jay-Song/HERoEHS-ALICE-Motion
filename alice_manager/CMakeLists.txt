################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(alice_manager)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED
  roscpp
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_controller_msgs
  robotis_math
  cmake_modules
  heroehs_math
  #heroehs_online_walking_pattern_generator
  #heroehs_pd_balance_controller
  alice_base_module
  alice_leg_module
  alice_balance_control
)

find_package(Eigen3 REQUIRED)

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
  LIBRARIES alice_manager
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(alice_manager_node src/alice_manager_node.cpp)

add_dependencies(alice_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(alice_manager_node
  ${catkin_LIBRARIES}
)
