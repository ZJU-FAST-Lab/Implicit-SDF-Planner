cmake_minimum_required(VERSION 3.0.2)
project(utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++14 -O3 -fPIC -pthread"  )

file(GLOB mylibigllib CONFIGURE_DEPENDS  "${CMAKE_SOURCE_DIR}/../lib/*.a") 

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
)
include_directories(${EIGEN3_INCLUDE_DIR})
add_message_files(FILES
  debug.msg
)
generate_messages(DEPENDENCIES
  std_msgs
)
catkin_package(
   INCLUDE_DIRS include ${TORCH_INCLUDE_DIRS}
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS nodelet roscpp rospy std_msgs message_runtime pcl_ros pcl_conversions

)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
   src/Shape.cpp
   src/debug_publisher.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  "${mylibigllib}"
)



