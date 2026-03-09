cmake_minimum_required(VERSION 3.0.2)
project(flight_mission)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
cv_bridge 
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  livox_ros_driver
)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc imgcodecs)

catkin_package(
  INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



add_executable(flight_mission 
  src/main.cpp
  src/mission_globals.cpp
  src/mission_callbacks.cpp
  src/flight_control.cpp
  src/obstacle_avoidance.cpp
  src/ring_detection.cpp
  src/vision_detection.cpp
  src/mission_tools.cpp
  src/map_class.cpp
)

target_link_libraries(flight_mission ${catkin_LIBRARIES}  ${OpenCV_LIBS})
