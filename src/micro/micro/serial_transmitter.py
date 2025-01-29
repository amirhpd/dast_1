#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SerialTransmitter(Node):
    def __init__(self):
        super().__init__("serial_transmitter")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.sub_ = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)
        self.sub_
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

    def msgCallback(self, msg):
        self.get_logger().info("Publishing on serial: %s" % self.arduino_.name)
        self.arduino_.write(msg.data.encode("utf-8"))


def main():
    rclpy.init()

    serial_transmitter = SerialTransmitter()
    rclpy.spin(serial_transmitter)
    
    serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# ros2 run micro serial_transmitter.py --ros-args -p port:=/dev/ttyACM0
# ros2 topic list  -> shows /serial_transmitter
# ros2 topic pub /serial_transmitter std_msgs/msg/String "data: '90'"  -> moves servo 90 degrees
