#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SerialReceiver(Node):
    def __init__(self):
        super().__init__("serial_receiver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.pub_ = self.create_publisher(String, "serial_receiver", 10)
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline()

            try:
                data.decode("utf-8")
            except:
                return

            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)


def main():
    rclpy.init()

    serial_receiver = SerialReceiver()
    rclpy.spin(serial_receiver)
    
    serial_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# ros2 run micro serial_receiver.py --ros-args -p port:=/dev/ttyACM0
# ros2 topic list  -> shows /serial_receiver
# ros2 topic echo /serial_receiver
