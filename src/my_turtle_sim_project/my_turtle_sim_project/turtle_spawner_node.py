#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import random
from functools import partial

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from my_robot_interfaces.msg import AliveTurtles
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_spawner") # MODIFY NAME
        self.declare_parameter("spawn_time", 1.5)

        self.spawn_time_ = self.get_parameter("spawn_time").value
        self.counter_ = 0
        self.alive_turtles_ = []
        self.publisher_ = self.create_publisher(AliveTurtles, "alive_turtles", 10)
        
        self.timer_ = self.create_timer(1/self.spawn_time_, self.spawn_turtles)

        self.server_ = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle_service)
        self.get_logger().info("Catch turtle server has been started.")

    def call_kill_turtle_service(self, name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting kill turtle service")

        request = Kill.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill_turtle_service, name=name))

    def callback_kill_turtle_service(self, future, name):
        try:

            response = future.result()
            self.get_logger().info("Turtle " + name + " has been killed")
            for (i, Turtle) in enumerate(self.alive_turtles_):
                if Turtle.name == name:
                    del self.alive_turtles_[i]
                    self.alive_tutler_publisher()
                    break
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def callback_catch_turtle_service(self, request, response):


        self.call_kill_turtle_service(request.name)
        response.success = True
        
        return response

    def alive_tutler_publisher(self):
        msg = AliveTurtles()
        msg.turtles = self.alive_turtles_
        self.publisher_.publish(msg)


    def spawn_turtles(self):
        x = random.uniform(1.0, 9.5)
        y = random.uniform(1.0, 9.5)
        theta = random.uniform(0.0, 360.0)
        self.call_turtle_spawn_server(x,y,theta)
        self.counter_ += 1

    
    def call_turtle_spawn_server(self, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for turtle spawn server...")

        request = Spawn.Request()
        request.x = x 
        request.y = y 
        request.theta = theta 

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_turtle_spawn, x=x, y=y, theta=theta))

    def callback_call_turtle_spawn(self, future, x, y, theta):
        try:

            response = future.result()
            self.get_logger().info("Turtle " + response.name + " is spawned")
            
            new_turtle = Turtle()
            new_turtle.x = x
            new_turtle.y = y
            new_turtle.theta = theta
            new_turtle.name = response.name
            self.alive_turtles_.append(new_turtle)
            self.alive_tutler_publisher()


        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
