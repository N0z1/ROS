cmake_minimum_required(VERSION 2.8.3)
project(ros_wall_traveller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf

)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_wall_traveller src/ros_wall_traveller.cpp)
target_link_libraries(ros_wall_traveller ${catkin_LIBRARIES})
