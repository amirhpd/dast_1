import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Pass gui:=False rviz:=False to run without any windows -- faster to start and
    # enough for anything driven over topics/actions rather than watched on screen.
    gui_arg = DeclareLaunchArgument("gui", default_value="True")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="True")

    gazebo_launch = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("description"),
                "launch",
                "gazebo.launch.py"
            ),
            launch_arguments={"gui": LaunchConfiguration("gui")}.items()
        )
    
    moveit_launch = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "True",
                              "rviz": LaunchConfiguration("rviz")}.items()
        )
    
    task_launch = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("task"),
                "launch",
                "task_interface.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
    
    return LaunchDescription([
        gui_arg,
        rviz_arg,
        gazebo_launch,
        moveit_launch,
        task_launch,
    ])

# ros2 launch startup sim_robot.launch.py
# ros2 launch startup sim_robot.launch.py gui:=False rviz:=False   # headless
# ros2 action list  -> shows /task_server_angle
# ros2 action send_goal /task_server_angle msgs/action/TaskAction "task_number: 0"

