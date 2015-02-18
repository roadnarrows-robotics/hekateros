cmake_minimum_required(VERSION 2.8.3)
project(hekateros_control)

find_package(catkin 
  REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs
  industrial_msgs
  sensor_msgs
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr_DIR /usr/local/lib/cmake/rnr)
find_package(rnr
  REQUIRED
)

catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES 
    Gpio.msg 
    HekOpState.msg 
    ProductInfo.msg 
    HekJointStateExtended.msg 
    HekRobotStatusExtended.msg
    ServoHealth.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    Calibrate.srv 
    ClearAlarms.srv 
    CloseGripper.srv 
    ConfigGpio.srv 
    EStop.srv 
    Freeze.srv 
    GetProductInfo.srv 
    GotoBalancedPos.srv 
    GotoParkedPos.srv 
    GotoZeroPt.srv 
    IsAlarmed.srv 
    IsCalibrated.srv 
    IsDescLoaded.srv
    OpenGripper.srv
    ReadGpio.srv 
    Release.srv
    ReloadConfig.srv
    ResetEStop.srv 
    SetRobotMode.srv
    Stop.srv
    WriteGpio.srv
)

add_action_files(
  DIRECTORY action 
  FILES 
    Calibrate.action
)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  industrial_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

add_executable(hekateros_control
  src/hekateros_as_calib.cpp
  src/hekateros_as_follow_traj.cpp
  src/hekateros_control.cpp
  src/hekateros_control_main.cpp
)

add_executable(hek_teleop 
  src/hek_teleop/hek_teleop.cpp 
  src/hek_teleop/hek_teleop_main.cpp 
)

include_directories(
  include 
  ${rnr_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(hekateros_control 
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(hek_teleop 
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(hekateros_control hekateros_control_gencpp)
add_dependencies(hek_teleop hekateros_control hek_control_gencpp)

install(
  PROGRAMS 
    scripts/calibrate
    scripts/hek_panel
    scripts/move_arm
    scripts/reload_config
    scripts/trajectory
    #scripts/hek_pyteleop
    #scripts/hek_teleop_viewer
    #scripts/joint_trajectory_demo 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
