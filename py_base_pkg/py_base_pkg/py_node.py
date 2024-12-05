#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class PyNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("node_name") # MODIFY NAME
        self.counter_ = 0
        self.get_logger().info("hello ROS2")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ +=1
        self.get_logger().info("hello " + str(self.counter_))


def main(args=None):
    rclpy.init(args=args)
    node = PyNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
