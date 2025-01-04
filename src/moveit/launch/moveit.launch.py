# Launch file to execute moveit.
# commands to run:
# ros2 launch description gazebo.launch.py
# ros2 launch moveit moveit.launch.py
# to move the robot:
# ros2 launch description gazebo.launch.py  # This will also start the conroller launch file 
# ros2 launch moveit moveit.launch.py 
# ros2 run moveit interface_pose 2.094 -1.345 0.038 -3.053 0.000 1.000
# or
# ros2 run moveit interface_angle 1.0 0.52 1.57 1.57 0.0


from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
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

    # moveit node
    moveit_config = (
        MoveItConfigsBuilder("dast_1", package_name="moveit")
        .robot_description(file_path=f"{get_package_share_directory('description')}/urdf/description.urdf.xacro")
        .robot_description_semantic(file_path="config/dast_1.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": is_sim_param}, {"publish_robot_description_semantic": True}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # rviz node
    rviz_node = Node(
        package="rviz2",
        name="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", f"{get_package_share_directory('moveit')}/config/moveit.rviz" ],  # [1]
        parameters=[
            # moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    return LaunchDescription([
        is_sim_arg,
        move_group_node,
        rviz_node
    ])


# [1]
# comment out this line first and run rviz once, set the following:
# # Fixed Frame: world
# # add MotionPlanning
# # Context -> Planning Library: ompl --- if not there: sudo apt-get install ros-humble-moveit-planners
# # Planning -> check Aprrox IK Solutions
# # Planning -> Execute: manipulator
# save the config in moveit/config/moveit.rviz
