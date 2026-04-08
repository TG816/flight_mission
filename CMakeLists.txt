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
  quadrotor_msgs
)

find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc imgcodecs objdetect dnn)

# ==============================================
# 自动查找 ONNX Runtime（不管装在哪都能找到）
# ==============================================
find_path(ONNX_INCLUDE_DIR
  NAMES onnxruntime_c_api.h
  PATHS /usr/local /opt /usr /home
  PATH_SUFFIXES onnxruntime/include include
)

find_library(ONNX_LIBRARY
  NAMES onnxruntime
  PATHS /usr/local /opt /usr /home
  PATH_SUFFIXES onnxruntime/lib lib
)

# ==============================================
# 自动查找 QR 码库 quirc（自动找，不用写路径）
# ==============================================
find_library(QR_LIBRARY
  NAMES quirc
  PATHS /usr/local /opt /usr
)

catkin_package(
  INCLUDE_DIRS include
)

# 头文件包含
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ONNX_INCLUDE_DIR}
)

# 编译可执行文件
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

# 链接库（全部自动找到）
target_link_libraries(flight_mission 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${ONNX_LIBRARY}
  ${QR_LIBRARY}
)

# 运行时自动找到 ONNX 库（不报错）
get_filename_component(ONNX_LIB_DIR ${ONNX_LIBRARY} DIRECTORY)
set_target_properties(flight_mission PROPERTIES
  INSTALL_RPATH ${ONNX_LIB_DIR}
  BUILD_WITH_INSTALL_RPATH ON
)