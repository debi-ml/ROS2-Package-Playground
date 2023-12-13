#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool



class NumberCounterNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("number_counter") # MODIFY NAME
        
        self.counter_ = 0
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_number_counter, 10)
        self.get_logger().info("Number Counter Node has started")

        self.server_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter)
        self.get_logger().info("Reset counter server has been started.")

    def callback_number_counter(self,msg):
        new_msg = Int64()
        self.counter_ = self.counter_ + msg.data
        new_msg.data = self.counter_
        self.publisher_.publish(new_msg)
        self.get_logger().info(str(new_msg.data))
    
    def callback_reset_counter(self, request, response):
        if request.data:
            self.counter_ = 0
            response.success = True
            response.message = "Counter has been successfuly reset"
        else: 
            response.success = False
            response.message = "Fail"
        return response






def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
