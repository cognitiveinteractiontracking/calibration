cmake_minimum_required(VERSION 2.8.3)
project(calib)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
  dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/Transform.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES genicam arghandler
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs tf tf2 dynamic_reconfigure
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(one_shot_calib_node src/one_shot_calib_node.cpp)

target_link_libraries(one_shot_calib_node
  ${catkin_LIBRARIES}
)

add_executable(dynamic_transform_publisher src/dynamic_transform_publisher_node.cpp)

target_link_libraries(dynamic_transform_publisher
  ${catkin_LIBRARIES}
)

add_dependencies(dynamic_transform_publisher ${PROJECT_NAME}_gencfg)
