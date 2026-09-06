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
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get the URDF file as an argument
    description_dir = get_package_share_directory("description")
    robot_description_arg = DeclareLaunchArgument(
        name="robot_description",
        default_value=f"{description_dir}/urdf/description.urdf.xacro",
        description="Path to the URDF file.",
    )
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="True",
        description="Start the Gazebo GUI. Set to False for a headless, server-only run.",
    )
    gui_param = LaunchConfiguration("gui")

    robot_description_param = ParameterValue(
        Command([
            "xacro ", LaunchConfiguration("robot_description"),
            " is_sim:=", "True",
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
                     "use_sim_time": True}]
    )

    # Gazebo, with or without its GUI. Headless (-s) starts the physics server only:
    # much faster, and enough for anything that reads topics/actions rather than looking.
    #
    # worlds/dast_1.sdf is Gazebo's empty.sdf plus the Sensors system. Without that
    # system the Kinect is created and advertises its topics but never renders a
    # single frame, silently -- see the comment in that file.
    gz_source = PythonLaunchDescriptionSource(
        [f"{get_package_share_directory('ros_gz_sim')}/launch", "/gz_sim.launch.py"])

    world_file = f"{description_dir}/worlds/dast_1.sdf"

    gazebo_launch = IncludeLaunchDescription(  # load launch file in launch file
                gz_source,
                launch_arguments=[("gz_args", [f" -v 4 -r {world_file} "])],
                condition=IfCondition(gui_param),
             )

    gazebo_launch_headless = IncludeLaunchDescription(
                gz_source,
                launch_arguments=[("gz_args", [f" -s -v 4 -r {world_file} "])],
                condition=UnlessCondition(gui_param),
             )
    
    ros_gz_sim_node = Node(  # starts Gazebo sim and spawns the model
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "dast_1"],
    )

    # Network bridge of messages between ROS and Gazebo. The Kinect topics are
    # renamed to exactly what the real driver publishes, so RViz, MoveIt and
    # anything else downstream cannot tell the two modes apart.
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/kinect/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/kinect/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/kinect/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/kinect/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/kinect/image", "/kinect/rgb/image_raw"),
            ("/kinect/depth_image", "/kinect/depth/image_raw"),
            ("/kinect/camera_info", "/kinect/rgb/camera_info"),
        ],
    )

    controller_launch = IncludeLaunchDescription(  # load the controller
            PythonLaunchDescriptionSource([f"{get_package_share_directory('controller')}/launch", "/controller.launch.py"]),
            )

    return LaunchDescription([
        robot_description_arg,
        gui_arg,
        gazebo_resource_path_env_var,
        robot_state_publisher_node,
        gazebo_launch,
        gazebo_launch_headless,
        ros_gz_sim_node,
        gz_ros2_bridge,
        controller_launch
    ])