#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces import LedState
from my_robot_interfaces import SetLed

from functools import partial


class BatteryClientNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("battery_client")  # MODIFY NAME
        self.battery_state_ = 1
        self.last_time_battery_state_changed_ = self.get_current_time()
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)
        self.get_logger().info("Battery Node has been started...")

    def get_current_time(self):
        s, ns = self.get_clock().now().seconds_nanoseconds()
        time = s + ns / 1000000000.0
        return time

    def check_battery_state(self):
        time_now = self.get_current_time()
        if self.battery_state_ == 1:
            if time_now - self.last_time_battery_state_changed_ > 4.0:
                self.battery_state_ = 0
                self.get_logger().info("Battery empty. Recharging...")
                self.last_time_battery_state_changed_ = time_now
                self.call_led_panel_server(3, 0)
        else:
            if time_now - self.last_time_battery_state_changed_ > 6.0:
                self.battery_state_ = 1
                self.get_logger().info("Battery Reacharged!!!")
                self.last_time_battery_state_changed_ = time_now
                self.call_led_panel_server(3, 1)

    def call_led_panel_server(self, led_number, state):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server LED panel")

        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_set_states, led_number=led_number, state=state))

    def callback_set_states(self, future, led_number, state):
        try:

            response = future.result()
            self.get_logger().info(str(response.message))

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryClientNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
