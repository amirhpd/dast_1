cmake_minimum_required(VERSION 3.8)
project(micro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(PkgConfig)
# pkg_check_modules(SERIAL libserial)

ament_python_install_package(${PROJECT_NAME})


install(PROGRAMS
  ${PROJECT_NAME}/serial_transmitter.py
  ${PROJECT_NAME}/serial_receiver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
