cmake_minimum_required(VERSION 2.8.3)
project(hekateros_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

# RDK, DHP TODO Create rnrpkgConfig.cmake
find_library(
  rnr_LIBRARIES
  NAMES hekateros Dynamixel botsense netmsgs rnr_appkit rnr
  PATHS /prj/lib /prj/lib/rnr
)

## System dependencies are found with CMake's conventions
## this is where rnr deps will go
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES Version.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES QueryVersion.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
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
#  INCLUDE_DIRS include
#  LIBRARIES hekateros_control
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include /prj/include ${catkin_INCLUDE_DIRS})

add_executable(hekateros_control src/hekateros_control.cpp)
target_link_libraries(
  hekateros_control 
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(hekateros_control hekateros_control_gencpp)

#add_executable(hekateros_control_client src/hekateros_control_client.cxx)
#target_link_libraries(hekateros_control_client ${catkin_LIBRARIES})
#add_dependencies(hekateros_control_client hekateros_control_client_gencpp)

#add_executable(hekateros_control_node src/hekateros_control_node.cxx)
#target_link_libraries(hekateros_control_node ${catkin_LIBRARIES})
