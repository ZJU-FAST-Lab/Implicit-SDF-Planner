cmake_minimum_required(VERSION 3.0.2)
project(sdf_vis)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++14 -O3 -fPIC"  )
# set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sdf_vis
 CATKIN_DEPENDS  roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)