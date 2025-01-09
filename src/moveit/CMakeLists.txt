cmake_minimum_required(VERSION 3.8)
project(moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(interface_angle src/interface_angle.cpp)
ament_target_dependencies(interface_angle rclcpp moveit_ros_planning_interface)

add_executable(interface_pose src/interface_pose.cpp)
ament_target_dependencies(interface_pose rclcpp moveit_ros_planning_interface)

add_executable(find_reachable_workspace src/find_reachable_workspace.cpp)
ament_target_dependencies(find_reachable_workspace rclcpp moveit_ros_planning_interface pcl_ros)

install(TARGETS
interface_angle interface_pose find_reachable_workspace
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS
  moveit/publish_pointcloud.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
