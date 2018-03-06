cmake_minimum_required(VERSION 2.8.3)
project(light)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment if the package has a setup.py
catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AddressColors.msg
  LightChange.msg
  CCT.msg
  Intensity.msg
  RAGBW.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  LightState.srv
  TestLightChange.srv
  PentaLight_CCT.srv
  PentaLight_int.srv
  PentaLight_ragbw.srv
  GetCCT.srv
  GetInt.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
#  light
)

catkin_package(
 # CATKIN_DEPENDS message_runtime
)
