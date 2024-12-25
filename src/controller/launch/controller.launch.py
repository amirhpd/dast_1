# Launch file to run the controler
# This launch file is loaded within the gazebo.launch.py file. 
# Alternatively, Gazebo and controller can be run separately:
# # ros2 launch description gazebo.launch.py 
# # ros2 launch controller controller.launch.py
# To see the running controllers using ROS2 CLI:
# # ros2 control list_controllers
# # ros2 control list_hardware_components
# # ros2 control list_hardware_interfaces


from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
    # Get is_sim as an argument
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )
    is_sim_param = LaunchConfiguration("is_sim")
    # nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        condition=UnlessCondition(is_sim_param),
        parameters=[{"robot_description": robot_description_param}],
    )

    controller_manager__spawner__joint_state_broadcaster__node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    controller_manager__spawner__manipulator_controller__node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller", "--controller-manager", "/controller_manager"],
    )

    # controller_manager__spawner__gripper_controller__node = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    # )

    return LaunchDescription([
        robot_description_arg,
        is_sim_arg,
        robot_state_publisher_node,
        controller_manager__spawner__joint_state_broadcaster__node,
        controller_manager__spawner__manipulator_controller__node
    ])