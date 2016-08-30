cmake_minimum_required(VERSION 2.8.12)
project(state_control_vehicle)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  nav_msgs 
  sensor_msgs 
  geometry_msgs 
  quadrotor_msgs 
  std_msgs 
  std_srvs
  mav_manager 
  cmake_modules)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mav_manager
  CATKIN_DEPENDS 
    roscpp 
    nav_msgs 
    sensor_msgs 
    geometry_msgs 
    quadrotor_msgs 
    trackers_manager 
    std_trackers 
    std_msgs 
    mav_manager
  DEPENDS Eigen)

add_executable(state_control src/state_control.cpp src/trajectory.cpp)
target_link_libraries(state_control ${catkin_LIBRARIES})
