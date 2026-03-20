cmake_minimum_required(VERSION 3.0.2)
project(flight_mission)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
)

# 补充 PCL 查找
find_package(PCL REQUIRED)

# 查找 OpenCV4
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc imgcodecs objdetect dnn)

# ========== 新增：查找 tesseract 库 ==========
find_library(TESSERACT_LIB tesseract)
if(NOT TESSERACT_LIB)
  message(FATAL_ERROR "tesseract库未找到！请执行：sudo apt install libtesseract-dev")
endif()

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # 新增：tesseract 头文件路径（一般安装后会自动识别，无需手动加）
  /usr/include/tesseract
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

# ========== 新增：链接 tesseract 库 ==========
target_link_libraries(flight_mission 
  ${catkin_LIBRARIES}  
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${TESSERACT_LIB}  # 链接 tesseract 库
)
