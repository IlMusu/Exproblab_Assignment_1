cmake_minimum_required(VERSION 3.0.2)
project(exprob_assignment_1)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  robot_state_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. 
# catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

## The catkin_package macro generates cmake config files for your package.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exprob_assignment_1
   CATKIN_DEPENDS rospy std_msgs robot_state_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Declare runnables
catkin_install_python(
  PROGRAMS
    scripts/robot_behaviour_helper.py
    scripts/robot_behaviour.py
    scripts/robot_state.py
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
