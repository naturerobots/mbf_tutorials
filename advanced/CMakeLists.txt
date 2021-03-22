cmake_minimum_required(VERSION 2.8.3)
project(mbf_advanced)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

catkin_install_python(
  PROGRAMS scripts/circle_smach.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
