cmake_minimum_required(VERSION 3.8)
project(whycon CXX)

# Plain CMake build (no colcon/ament). Source your ROS 2 setup.bash first so
# that find_package can locate rclcpp and the message packages.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(whycon
  main/whycon.cpp
  common/CRawImage.cpp
  common/CTimer.cpp
  common/cmath.cpp
  imageproc/CCircleDetect.cpp
  imageproc/CTransformation.cpp
  imageproc/sysmat.cpp
)
target_include_directories(whycon PRIVATE common imageproc)
target_link_libraries(whycon
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

# the program expects to be run from bin/ (it reads ../etc/Calib_Results.m)
set_target_properties(whycon PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../bin)
