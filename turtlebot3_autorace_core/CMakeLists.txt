################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_autorace_core)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package()

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  nodes/core_node_mission
  nodes/construction_core_mode_decider
  nodes/construction_core_node_controller
  nodes/intersection_core_mode_decider
  nodes/intersection_core_node_controller
  nodes/level_crossing_core_mode_decider
  nodes/level_crossing_core_node_controller
  nodes/parking_core_mode_decider
  nodes/parking_core_node_controller
  nodes/traffic_light_core_mode_decider
  nodes/traffic_light_core_node_controller
  nodes/tunnel_core_mode_decider
  nodes/tunnel_core_node_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
