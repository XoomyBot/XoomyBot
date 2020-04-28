################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(wheel_control)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Add messages
################################################################################
add_message_files(
  FILES
  path_message.msg
)

add_service_files(
  FILES
  path_service.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs message_runtime
)



