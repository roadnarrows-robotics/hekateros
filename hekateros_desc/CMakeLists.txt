cmake_minimum_required(VERSION 2.8.3)
project(hekateros_desc)

catkin_package()

install(DIRECTORY robots DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY stl DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
