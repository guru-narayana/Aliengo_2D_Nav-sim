cmake_minimum_required(VERSION 2.8.3)
project(keyboard_control)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

