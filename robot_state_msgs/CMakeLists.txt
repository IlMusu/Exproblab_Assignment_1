cmake_minimum_required(VERSION 3.0.2)
project(robot_state_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  Message1.msg
#  Message2.msg
#)

# Generate services in the 'srv' folder
add_service_files(
  FILES
    RoomPosition.srv
    ReferenceName.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
    ComputePath.action
    FollowPath.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_state_msgs
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    actionlib_msgs
    message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


