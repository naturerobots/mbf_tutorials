cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_mbf)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

catkin_install_python(PROGRAMS scripts/mb_relay.py scripts/mb_relay_a_to_b.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
