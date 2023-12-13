#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces import LedState
from my_robot_interfaces import SetLed


class LedPanelNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("led_panel_server")  # MODIFY NAME
        self.declare_parameter("led_states", [0,0,0])

        self.led_state_ = self.get_parameter("led_states").value
        self.publisher_ = self.create_publisher(LedState, "led_states", 10)
        self.timer_ = self.create_timer(3.0, self.publish_led_state)
        self.get_logger().info("Led State published has been started...")

        self.server_ = self.create_service(
            SetLed, "set_led", self.callback_set_led)
        self.get_logger().info("LED server is running.")

    def publish_led_state(self):
        msg = LedState()
        msg.led_state = self.led_state_
        self.publisher_.publish(msg)

    def callback_set_led(self, request, response):
        msg = LedState()
        led_number = request.led_number
        state = request.state

        if  led_number > len(self.led_state_) or led_number <= 0:
            response.success = False
            response.message = "Led number not in range. Valid range [1,3]"
            return response
        if state not in [0,1]:
            response.success = False
            response.message = "State value not in range. Valid range 0 or 1"
            return response
        self.led_state_[led_number-1] = state
        response.success = True
        response.message = "LED state change success"
        msg.led_state = self.led_state_
        self.publisher_.publish(msg)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
