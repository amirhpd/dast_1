cmake_minimum_required(VERSION 3.8)
project(task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclpy REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(task_server_angle SHARED src/task_server_angle.cpp)
target_include_directories(task_server_angle PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(task_server_angle
  PRIVATE "TASK_SERVER_ANGLE_CPP_BUILDING_DLL")
ament_target_dependencies(task_server_angle
  "msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
  "yaml-cpp")
rclcpp_components_register_node(task_server_angle 
  PLUGIN "task::TaskServerAngle" 
  EXECUTABLE task_server_angle_node
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  task_server_angle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  # ${PROJECT_NAME}/some_python_file.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
