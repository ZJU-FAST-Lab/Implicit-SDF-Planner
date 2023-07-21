cmake_minimum_required(VERSION 3.0.2)
project(map_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  roslib 
  utils
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++14 -O3 -fPIC"  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs roslib utils
)

add_library(map_manager_lib
  src/Gridmap3D.cpp
  src/PCSmap_manager.cpp 
)
target_link_libraries(map_manager_lib ${PCL_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable( globalmap_gene_node 
                src/globalmap_gene.cpp)

target_link_libraries(globalmap_gene_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)


