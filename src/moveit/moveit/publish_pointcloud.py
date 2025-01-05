#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField
from pypcd4 import PointCloud
import struct
import numpy as np


class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        # self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.publisher_ = self.create_publisher(PointCloud2, '/point_cloud', 30)
        self.timer = self.create_timer(1.0, self.publish_pcd)

    def publish_pcd(self):
        file_path = 'points.pcd'
        self.get_logger().info(f"Loading PCD file: {file_path}")

        # Load PCD file
        pc = PointCloud.from_path(file_path)
        points = pc.numpy(("x", "y", "z"))

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.frame_id = "world"
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12  # 4 bytes per point * 3 fields (x, y, z)
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.data = b''.join(struct.pack('fff', *point) for point in points)

        # Publish the message
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
