cmake_minimum_required(VERSION 3.0.2)
project(robot_behaviour)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
)

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
#add_action_files(
#  FILES
#  Action1.action
#  Action2.action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
      std_msgs
      actionlib_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_behaviour
  CATKIN_DEPENDS 
    rospy 
    std_msgs
    actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Define runnables.
catkin_install_python(
  PROGRAMS 
    scripts/robot_behaviour_helper.py 
    scripts/robot_behaviour_surveillance.py
    scripts/robot_stimulus_provider.py
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
