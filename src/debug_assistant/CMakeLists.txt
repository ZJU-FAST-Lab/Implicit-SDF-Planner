cmake_minimum_required(VERSION 3.0.2)
project(debug_assistant)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++14 -O3 -fPIC"  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
 CATKIN_DEPENDS  roscpp rospy std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

