#!/usr/bin/env python3
import rclpy
import random
import math

from rclpy.node import Node
from functools import partial
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim_interfaces.msg import Turtle
from turtlesim_interfaces.msg import TurtleArray
from turtlesim_interfaces.srv import CatchTurtle

class SpawnTurtleNode(Node):
    def __init__(self):
        super().__init__("spawn_turtle_node")
        self.name_ = "turtle"
        self.counter_ = 0
        self.new_turtles_ = []

        self.new_turtles_publisher_ = self.create_publisher(TurtleArray, "new_turtles", 10)
        self.timer = self.create_timer(1.0, self.spawn_turtle)
        self.call_catch_turtle_service_ = self.create_service(CatchTurtle, "/catch", self.callback_catch_turtle)

    def callback_catch_turtle(self, request, response):
        self.call_kill_server(request.name)
        response.success = True
        return response
    
    def publish_new_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.new_turtles_
        self.new_turtles_publisher_.publish(msg)

    def spawn_turtle(self):
        self.counter_ += 1
        turtle_name = self.name_ + str(self.counter_)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2 * math.pi)
        self.call_spawn_turtle_server(x, y, theta, turtle_name)

    def call_spawn_turtle_server(self, x, y, theta, turtle_name):
        client_ = self.create_client(Spawn, "/spawn")
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server - [Spawn Turtles]")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client_.call_async(request)
        future.add_done_callback(
            partial(
                self.callback_call_spawn_turtle,
                x=x,
                y=y,
                theta=theta,
                turtle_name=turtle_name
            )
        )

    def callback_call_spawn_turtle(self, future, x, y, theta, turtle_name):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info("Turtle: " + response.name + " is created")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.new_turtles_.append(new_turtle)
                self.publish_new_turtles()

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def call_kill_server(self, turtle_name):
        client_ = self.create_client(Kill, "/kill")
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server - [Kill Turtles]")

        request = Kill.Request()
        request.name = turtle_name

        future = client_.call_async(request)
        future.add_done_callback(
            partial(
                self.callback_call_kill_turtle, turtle_name=turtle_name))

    def callback_call_kill_turtle(self, future, turtle_name):
        try:
            response = future.result()
            for (i, turtle) in enumerate(self.new_turtles_):
                if turtle.name == turtle_name:
                    del self.new_turtles_[i]
                    self.publish_new_turtles()
                    break
 
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()