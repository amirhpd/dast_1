import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controller_launch = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    moveit_launch = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    task_launch = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("task"),
                "launch",
                "task_interface.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    return LaunchDescription([
        controller_launch,
        moveit_launch,
        task_launch,
    ])

# ros2 launch startup run_robot.launch.py
# ros2 action list  -> shows /task_server_angle
# ros2 action send_goal /task_server_angle msgs/action/TaskAction "task_number: 0" -> servos should move
