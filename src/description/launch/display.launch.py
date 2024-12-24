# Launch file to display the urdf. Runs the following commands:
# ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/amir/ros_projects/dast_1_ws/src/description/urdf/description.urdf.xacro)"
# ros2 run joint_state_publisher_gui joint_state_publisher_gui 
# ros2 run rviz2 rviz2 
# command to run: 
# ros2 launch description display.launch.py


from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the URDF file as an argument
    description_dir = get_package_share_directory("description")
    robot_description_arg = DeclareLaunchArgument(
        name="robot_description",
        default_value=f"{description_dir}/urdf/description.urdf.xacro",
        description="Path to the URDF file.",
    )
    robot_description_param = ParameterValue(
        Command(["xacro ", LaunchConfiguration("robot_description")]),
        value_type=str
    )
    # nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_param}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output='screen',
        arguments=["-d", f"{description_dir}/rviz/display.rviz"],  # [1]
    )

    return LaunchDescription([
        robot_description_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])

# [1]
# Run the ros2 run commands (run without launch file)
# Modify the rviz env:
# # set Fixed Frame to world
# # add RobotModel
# # in RobotModel, set the Description Topic to /robot_description
# # save as in /rviz folder with name display.rviz
# Now launch file works and loads the saved rviz configuration