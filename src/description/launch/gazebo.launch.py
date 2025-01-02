# Launch file to start the Gazebo for the URDF model.
# Shows the robot in Gazebo, but does not move without the controller.
# command to run: 
# ros2 launch description gazebo.launch.py

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        Command([
            "xacro ", LaunchConfiguration("robot_description"),
            " is_ignition:=", "True",  # for Humble: "True"
            ]),
        value_type=str
    )
    # Set env vars 
    gazebo_resource_path_env_var = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(description_dir).parent.resolve())
            ]
        )
    
    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_param,
                     "use_sim_time": True}],
        # ros_arguments=["joint_states:=/joint_states"]
    )

    gazebo_launch = IncludeLaunchDescription(  # load launch file in launch file
                PythonLaunchDescriptionSource([f"{get_package_share_directory('ros_gz_sim')}/launch", "/gz_sim.launch.py"]),
                launch_arguments=[("gz_args", [" -v 4 -r empty.sdf ", ""])]  # for Humble: ""
             )
    
    ros_gz_sim_node = Node(  # starts Gazebo sim and spawns the model
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "dast_1"],
        parameters=[{"use_sim_time": True}]
    )

    gz_ros2_bridge = Node(  # network bridge of messages between ROS and Gazebo
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}]
    )

    controller_launch = IncludeLaunchDescription(  # load the controller
            PythonLaunchDescriptionSource([f"{get_package_share_directory('controller')}/launch", "/controller.launch.py"]),
            )

    return LaunchDescription([
        robot_description_arg,
        gazebo_resource_path_env_var,
        robot_state_publisher_node,
        gazebo_launch,
        ros_gz_sim_node,
        gz_ros2_bridge,
        controller_launch
    ])