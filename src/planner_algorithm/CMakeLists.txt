cmake_minimum_required(VERSION 3.0.2)
project(planner_algorithm)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 --std=c++14 -fPIC -fopenmp"  )
set(CMAKE_BUILD_TYPE "Release")
add_library(lmbm SHARED IMPORTED)
set_target_properties(lmbm PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../utils/include/lmbm/lmbm.so)

find_package(catkin REQUIRED COMPONENTS
  map_manager
  geometry_msgs
  roscpp
  rospy
  swept_volume
  sensor_msgs
  std_msgs
  utils
  swept_volume
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS map_manager geometry_msgs roscpp rospy sensor_msgs std_msgs swept_volume utils
)
file(GLOB myheaders CONFIGURE_DEPENDS  "${CMAKE_SOURCE_DIR}/planner_algorithm/include/planner_algorithm/*.hpp") 
add_library(planner_algorithm_lib
src/back_end_optimizer.cpp
src/mid_end.cpp
${myheaders}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(planner_algorithm_lib PUBLIC
  OpenMP::OpenMP_CXX
  lmbm
)


