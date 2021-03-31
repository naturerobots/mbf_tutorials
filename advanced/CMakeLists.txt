cmake_minimum_required(VERSION 2.8.3)
project(mbf_advanced)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(BehaviorTreeV3)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  actionlib
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  DEPENDS move_base_flex
)

catkin_install_python(
  PROGRAMS scripts/circle_smach.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(behavior_tree src/behavior_tree.cpp)
target_include_directories(behavior_tree PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(behavior_tree BT::behaviortree_cpp_v3 ${catkin_LIBRARIES})
target_compile_definitions(behavior_tree
        PRIVATE
        BT_XML_PATH="${PROJECT_SOURCE_DIR}/param/behavior_tree.xml"
        POSE_PATH="${PROJECT_SOURCE_DIR}/param/poses.txt"
)

add_dependencies(behavior_tree ${behavior_tree_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mbf_cpp_client_node src/mbf_cpp_client_node.cpp)
target_include_directories(mbf_cpp_client_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(mbf_cpp_client_node ${catkin_LIBRARIES})
target_compile_definitions(mbf_cpp_client_node
        PRIVATE
        POSE_PATH="${PROJECT_SOURCE_DIR}/param/poses.txt"
)