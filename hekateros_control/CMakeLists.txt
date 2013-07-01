cmake_minimum_required(VERSION 2.8.3)
project(hekateros_control)

find_package(catkin 
  REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs
  genmsg 
  message_generation 
  roscpp 
  rospy 
  std_msgs 
)

find_package(Boost REQUIRED COMPONENTS system)

# RDK, DHP TODO Create rnrpkgConfig.cmake
# find_package(rnr
#   REQUIRED COMPONENTS hekateros Dynamixel botsense netmsgs rnr_appkit rnr
# )
find_library(
  rnr_LIBRARIES
  NAMES hekateros Dynamixel botsense netmsgs rnr_appkit rnr
  PATHS /prj/lib /prj/lib/rnr
)

catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES Version.msg
)

add_service_files(
  DIRECTORY srv
  FILES GetVersion.srv
)

add_action_files(
  DIRECTORY action 
  FILES Calibrate.action
)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  hekateros_control
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include 
  /prj/include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(hekateros_control src/hekateros_control.cpp)
add_executable(hekateros_control_client src/hekateros_control_client.cxx)

install(
  PROGRAMS scripts/test 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(
  hekateros_control 
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  hekateros_control_client
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(hekateros_control hekateros_control_gencpp)
add_dependencies(hekateros_control_client hekateros_control_gencpp)

#add_executable(hekateros_control_client src/hekateros_control_client.cxx)
#target_link_libraries(hekateros_control_client ${catkin_LIBRARIES})
#add_dependencies(hekateros_control_client hekateros_control_client_gencpp)

#add_executable(hekateros_control_node src/hekateros_control_node.cxx)
#target_link_libraries(hekateros_control_node ${catkin_LIBRARIES})
