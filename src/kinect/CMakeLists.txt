cmake_minimum_required(VERSION 3.8)
project(kinect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# libfreenect ships no CMake config on Debian/Ubuntu, only a .pc file, and the
# sync helper lives in a second library that the .pc does not mention.
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREENECT REQUIRED libfreenect)
find_library(FREENECT_SYNC_LIB NAMES freenect_sync REQUIRED)

add_executable(kinect_node src/kinect_node.cpp)
target_include_directories(kinect_node PRIVATE
  "${FREENECT_INCLUDE_DIRS}"
)
target_link_libraries(kinect_node PUBLIC
  "${FREENECT_LDFLAGS}"
  "${FREENECT_SYNC_LIB}"
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
)

install(TARGETS
  kinect_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
