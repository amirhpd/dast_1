# Launch file to execute the task server by action.
# To run:
# ros2 launch description gazebo.launch.py 
# ros2 launch moveit moveit.launch.py 
# ros2 launch task task_interface.launch.py
# ros2 action list  -> shows /task_server_angle
# ros2 action send_goal /task_server_angle msgs/action/TaskAction "task_number: 0" 

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get is_sim as an argument
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )
    is_sim_param = LaunchConfiguration("is_sim")

    task_server_angle_node = Node(
        package="task",
        executable="task_server_angle_node",
        name="task_server_angle_node",
        parameters=[
            {"use_sim_time": is_sim_param},
            # "config/tasks.yaml"
            ]
    )

    return LaunchDescription([
        is_sim_arg,
        task_server_angle_node,
    ])
