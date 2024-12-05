#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64


class NumberSubscriberNode(Node):
    def __init__(self):
        super().__init__("number_subscriber")

        
        self.number_subscriber_ = self.create_subscription(
            Int64, "number_topic", self.callback_number, 10)
        self.get_logger().info("Number Counter has been started.")

    def callback_number(self, msg):
        self.counter_ += msg.data
        new_msg = Int64()
        new_msg.data = self.counter_
        self.number_count_publisher_.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
