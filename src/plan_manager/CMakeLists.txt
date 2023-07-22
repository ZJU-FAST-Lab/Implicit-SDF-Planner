cmake_minimum_required(VERSION 3.0.2)
project(plan_manager)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++14 -O3 -fPIC"  )

find_package(catkin REQUIRED COMPONENTS
  map_manager
  roscpp
  rospy
  utils
  swept_volume
  planner_algorithm
)


find_package(Eigen3 REQUIRED)
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy  utils  swept_volume planner_algorithm
    LIBRARIES ${PROJECT_NAME}
)
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} src/plan_manager.cpp)
add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   map_manager_lib
   planner_algorithm_lib
 )
