# aggressive planner by wangzichen
cmake_minimum_required(VERSION 3.0.2)
project(aggressive_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  roscpp
  rospy
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aggressive_planner
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

link_directories(${PCL_LIBRARY_DIRS})

add_executable(
        planner
        src/Controller.cpp
        src/Fsm.cpp
        src/GridMap.cpp
        src/Astar.cpp
        src/Bspline.cpp
        src/TrajOpt.cpp
)
target_link_libraries(
    planner
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

