cmake_minimum_required(VERSION 3.8)
project(moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(graph_msgs REQUIRED)
# find_package(geometric_shapes REQUIRED)

add_executable(interface_angle src/interface_angle.cpp)
ament_target_dependencies(interface_angle rclcpp moveit_ros_planning_interface)

add_executable(interface_pose src/interface_pose.cpp)
ament_target_dependencies(interface_pose rclcpp moveit_ros_planning_interface geometric_shapes moveit_visual_tools rviz_visual_tools)
# ament_export_dependencies(interface_pose geometric_shapes)

include_directories(${rviz_visual_tools_INCLUDE_DIRS}) 
# include_directories(${moveit_visual_tools_INCLUDE_DIRS}) 
include_directories(${graph_msgs_INCLUDE_DIRS}) 

target_link_libraries(interface_pose ${rviz_visual_tools_LIBRARIES}) 
target_link_libraries(interface_pose ${moveit_visual_tools_LIBRARIES}) 
# target_link_libraries(interface_pose ${geometric_shapes_LIBRARIES}) 

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_visual_tools
)

install(TARGETS
interface_angle interface_pose
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
