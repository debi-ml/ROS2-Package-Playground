#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_robot_interfaces.msg import AliveTurtles
from my_robot_interfaces.msg import Turtle

from functools import partial

from my_robot_interfaces.srv import CatchTurtle


from math import atan2, sqrt, pi


class TurtleControllerNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_controller")  # MODIFY NAME

        self.declare_parameter("catch_closest_turtle", True)

        self.catch_closest_turtle_ = self.get_parameter("catch_closest_turtle").value
        self.pose_ = None
        self.turtle_to_catch_ = None

        self.subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.alive_turtle_subscriber_ = self.create_subscription(
            AliveTurtles, "alive_turtles", self.callback_alive_turtles, 10)
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.get_logger().info("Controller node has started")
        self.controll_loop_ = self.create_timer(
            0.01, self.control_loop_callback)

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def call_catch_turtle_service(self, name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server catch turtle")

        request = CatchTurtle.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_catch_turtle_service, name=name))

    def callback_catch_turtle_service(self, future, name):
        try:

            response = future.result()
            #self.get_logger().info("Turtle " + name + " has been cought")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    error_x = turtle.x - self.pose_.x
                    error_y = turtle.y - self.pose_.y
                    distance = sqrt(error_x**2 + error_y**2)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle
            else:
                self.turtle_to_catch_ = msg.turtles[0]

    def control_loop_callback(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return

        target_x = self.turtle_to_catch_.x
        target_y = self.turtle_to_catch_.y

        # Calculate the error between the current and target positions
        error_x = target_x - self.pose_.x
        error_y = target_y - self.pose_.y

        # Calculate the distance from the current pose to the target
        distance = sqrt(error_x**2 + error_y**2)

        # Calculate the desired heading angle
        desired_heading = atan2(error_y, error_x)

        # Calculate the error between the current and desired heading angles
        error_heading = desired_heading - self.pose_.theta

        # Create a Twist message to send the velocity command
        vel_msg = Twist()

        # Set the linear and angular velocity
        if distance < 0.5:
            self.call_catch_turtle_service(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
        else:
            vel_msg.linear.x = 2 * distance

            if error_heading > pi:
                error_heading -= 2*pi
            elif error_heading < -pi:
                error_heading += 2*pi

            vel_msg.angular.z = 6 * error_heading

        # Publish the velocity command
        self.publisher_.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
