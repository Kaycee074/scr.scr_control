cmake_minimum_required(VERSION 2.8.3)
project(scr_control)

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
  PentaLight_ragbwAll.srv
  PentaLight_CCTAll.srv
  GetBlinds.srv
  GetCCT.srv
  GetInt.srv
  GetLights.srv
  BlindLift.srv
  BlindLiftAll.srv
  BlindTilt.srv
  BlindTiltAll.srv
  COSReadAll.srv
  COSReadOne.srv
  COSInteTime.srv
  HVAC_SetTemp.srv
  HVAC_SetFanSp.srv
  HVAC_SetEp.srv
  HVAC_SetBms.srv
  HVAC_GetTemp.srv
  HVAC_GetEp.srv
  HVAC_GetCO2.srv
  HVAC_GetRH.srv
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

########################################
# Enable C++ Time of Flight sensor CPP #
########################################

SET(CMAKE_CXX_COMPILER, "g++")
SET(CMAKE_CXX_FLAGS "-w -D_GLIBCXX_USE_CXX11_ABI=0")
link_directories(
    src/time_of_flight/lib/x86_64-linux-g++-4.6.3-glibc-2.15/iris/sensor-api/static
    src/time_of_flight/lib/x86_64-linux-g++-4.6.3-glibc-2.15/3rd_party/boost-1_55_0/static
)
add_executable(TOF src/time_of_flight/SCR/test.cpp)

target_include_directories(
    TOF PUBLIC
    src/time_of_flight/include/iris/sensor-api
    src/time_of_flight/include/3rd_party/boost-1_55_0
)
include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(
    TOF PUBLIC
    ${catkin_LIBRARIES}
    IrmaMatrix
    boost_system-mt
    boost_filesystem-mt
    boost_thread-mt
    boost_regex-mt
    boost_date_time-mt
    boost_chrono-mt
    boost_regex
    dl
    pthread
)

add_dependencies(TOF ${catkin_EXPORTED_TARGETS})