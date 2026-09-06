# Standalone Kinect v1 test launch. Starts nothing from the rest of DAST-1 --
# no robot, no controllers, no MoveIt -- so a failure here is the sensor or the
# driver, and nothing else.
#
# To run:
# ros2 launch kinect kinect.launch.py rviz:=True
# ros2 run rqt_image_view rqt_image_view /kinect/depth/image_raw
#
# ros2 launch kinect kinect.launch.py
# ros2 topic hz /kinect/rgb/image_raw
#
# config/kinect.rviz was written by hand and is meant to be regenerated: launch
# with rviz:=True, adjust the displays, then File > Save Config As over it.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="False",
        description="Start RViz with the point cloud and camera images."
    )
    rviz_param = LaunchConfiguration("rviz")

    pointcloud_arg = DeclareLaunchArgument(
        "publish_pointcloud",
        default_value="True",
        description="Build a coloured point cloud. Only published when something subscribes."
    )
    pointcloud_param = LaunchConfiguration("publish_pointcloud")

    tilt_arg = DeclareLaunchArgument(
        "tilt_degrees",
        default_value="-16",
        description="Tilt commanded at startup, -30..30, negative is head down. "
                    "Must match the tilt the extrinsic calibration was measured at."
    )
    tilt_param = LaunchConfiguration("tilt_degrees")

    kinect_node = Node(
        package="kinect",
        executable="kinect_node",
        name="kinect",
        parameters=[{
            "publish_pointcloud": pointcloud_param,
            "frame_id": "kinect_rgb_optical_frame",
            "tilt_degrees": tilt_param,
        }]
    )

    # The camera publishes in an optical frame (z forward, x right, y down).
    # RViz needs a fixed frame that exists in TF, so give it a plain kinect_link
    # (x forward, z up) to hang that off.
    optical_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="kinect_optical_frame_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "-1.5708", "--pitch", "0", "--yaw", "-1.5708",
            "--frame-id", "kinect_link",
            "--child-frame-id", "kinect_rgb_optical_frame",
        ]
    )

    rviz_config = os.path.join(
        get_package_share_directory("kinect"), "config", "kinect.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz_param)
    )

    return LaunchDescription([
        rviz_arg,
        pointcloud_arg,
        tilt_arg,
        kinect_node,
        optical_frame_tf,
        rviz_node,
    ])
