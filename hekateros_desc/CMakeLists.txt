cmake_minimum_required(VERSION 2.8.3)
project(hekateros_desc)

catkin_package()

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
