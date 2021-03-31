cmake_minimum_required(VERSION 2.8.3)
project(mbf_advanced)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(BehaviorTreeV3)
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

catkin_install_python(
  PROGRAMS scripts/circle_smach.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(behavior_tree src/behavior_tree.cpp)
target_link_libraries(behavior_tree BT::behaviortree_cpp_v3)
target_compile_definitions(behavior_tree
        PRIVATE
        BT_XML_PATH="${PROJECT_SOURCE_DIR}/param/behavior_tree.xml"
        POSE_PATH="${PROJECT_SOURCE_DIR}/param/poses.txt"
)
