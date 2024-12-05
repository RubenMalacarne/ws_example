#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from functools import partial

from interfaces_pkg.srv import SetLed

class BatteryNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("battery_name") # MODIFY NAME
        self.battery_state_ = "full"
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_satate) 
        self.last_time_battery_state_changed_  = self.get_current_time_seconds()
        self.get_logger().info("battery has been started")
        
    def get_current_time_seconds(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs /1000000000.0
    
    def check_battery_satate(self):
        time_now = self.get_current_time_seconds()
        if self.battery_state_ == "full":
            if time_now -self.last_time_battery_state_changed_ > 4.0:
                self.battery_state_ = "empty"  
                self.get_logger().info("battery is empty, we are charging...")

                self.last_time_battery_state_changed_ = time_now
                
                self.call_set_led_server(3,1)
        else:
            if time_now - self.last_time_battery_state_changed_ >6.0:
                self.battery_state_ = "full"
                self.get_logger().info("Battery is now full again")
                self.last_time_battery_state_changed_ = time_now
                
                self.call_set_led_server (3,0)
            
    
    def callback_call_set_led(self, future, led_number, state):
        try:
            response = future.result()
            self.get_logger().info(str(response.success))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
            
    def call_set_led_server(self, led_number, state):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Add Two Ints...")

        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_set_led, led_number=led_number, state=state))
 
    

def main(args=None):
    
    rclpy.init(args=args)
    node = BatteryNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
