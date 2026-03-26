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
  visualization_msgs
  quadrotor_msgs
)

# 强制找 OpenCV4，添加 objdetect（QR码解码）+ dnn（OpenCV内置ONNX可选）
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc imgcodecs objdetect dnn)

catkin_package(
  INCLUDE_DIRS include
)

# ========== 新增 ONNX Runtime 配置开始 ==========
# 1. 添加 ONNX Runtime 头文件路径
include_directories(
  /usr/local/onnxruntime/include
)
# 2. 添加 ONNX Runtime 库文件路径
link_directories(
  /usr/local/onnxruntime/lib
)
# ========== 新增 ONNX Runtime 配置结束 ==========

# 原有 include 配置（保留不动）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/onnxruntime/include
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

# ========== 修改 target_link_libraries：添加 onnxruntime 库 ==========
target_link_libraries(flight_mission 
  ${catkin_LIBRARIES}  
  ${OpenCV_LIBS}
  onnxruntime  # 新增：链接 ONNX Runtime 库
  quirc  # 新增：链接 libquirc.so
)

# ========== 可选：添加运行时库路径（防止运行时报找不到 libonnxruntime.so） ==========
set_target_properties(flight_mission PROPERTIES
  INSTALL_RPATH "/usr/local/onnxruntime/lib"
  BUILD_WITH_INSTALL_RPATH ON
)