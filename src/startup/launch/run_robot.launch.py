import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
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
    
    # The real Kinect. Publishes the same topics the ros_gz_bridge publishes in
    # simulation, in the same frames, so nothing downstream changes between modes.
    # kinect.launch.py is deliberately not included: it carries its own static
    # transform for the optical frame, which robot_state_publisher already
    # provides here from the URDF.
    kinect_node = Node(
        package="kinect",
        executable="kinect_node",
        name="kinect",
        parameters=[{
            "frame_id": "kinect_rgb_optical_frame",
            "tilt_degrees": -16,
            # The robot model is in decimetres; see point_scale in kinect_node.cpp.
            "point_scale": 10.0,
        }],
    )

    return LaunchDescription([
        controller_launch,
        kinect_node,
        moveit_launch,
        task_launch,
    ])

# ros2 launch startup run_robot.launch.py
# ros2 action list  -> shows /task_server_angle
# ros2 action send_goal /task_server_angle msgs/action/TaskAction "task_number: 0" -> servos should move
