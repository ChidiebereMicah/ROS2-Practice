#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from functools import partial
import math


class Turtle_Love_Node(Node):
    
    def __init__(self):
        super().__init__("turtle_love2")
        self.turtle_exists = False
        self.previous_y_pose = 0.0
        self.Cinderella_pose = 0.0
        self.Superman_pose = 0.0
        self.spawn_service_condition()
        self.cmd_vel_publisher_Cinderella = self.create_publisher(Twist, '/Cinderella/cmd_vel', 10)
        self.cmd_vel_publisher_Superman = self.create_publisher(Twist, '/Superman/cmd_vel', 10)
        self.pose_subscriber_Cinderella = self.create_subscription(Pose, "/Cinderella/pose", self.turtle_love_callback_Cinderella, 10)
        self.pose_subscriber_Superman = self.create_subscription(Pose, "/Superman/pose", self.turtle_love_callback_Superman,  10)
        self.get_logger().info("Turtle Cinderella has been spawn")

    def spawn_service_condition(self): #to ensure spawn service is not called more than once
        if not self.turtle_exists:
            self.call_spawn_service( 5.48, 3.0, 1*math.pi/4, 'Cinderella' )
            self.call_spawn_service( 5.52, 3.0, 3*math.pi/4, 'Superman' )
            self.turtle_exists = True

    def turtle_love_callback_Cinderella(self, pose:Pose):
        cmd = Twist()
        self.get_logger().info(f"Cinderella current position: x={pose.x}, y={pose.y}, theta={pose.theta}, previous pose={self.previous_y_pose}")
        if pose.y >= 6.0 and self.previous_y_pose < 6.0:
            self.previous_y_pose = pose.y
            cmd.linear.x = 1.61
            cmd.angular.z = abs(0.9) # for clockwise rotation
        elif pose.y >= 6.0 and self.previous_y_pose >= 6.0:
            self.previous_y_pose = pose.y
            self.Cinderella_pose = round(pose.x, 1)
            cmd.linear.x = 1.61
            cmd.angular.z = abs(0.9) # for clockwise rotation
            if self.Cinderella_pose < 5.6 and  self.Superman_pose > 5.3: #to make turtle to stop
                self.previous_y_pose = pose.y
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        elif pose.y < 6.0 and self.previous_y_pose < 6.0:
            self.previous_y_pose = pose.y
            cmd.linear.x = 2.0
            cmd.angular.z = 0.0
    
        self.cmd_vel_publisher_Cinderella.publish(cmd)
        
    def turtle_love_callback_Superman(self, pose:Pose):
        cmd = Twist()
        self.get_logger().info(f"Superman current position: x={pose.x}, y={pose.y}, theta={pose.theta}, previous pose={self.previous_y_pose}")
        if pose.y >= 6.0 and self.previous_y_pose < 6.0:
            self.previous_y_pose = pose.y
            cmd.linear.x = 1.61
            cmd.angular.z = -abs(0.9) # for anticlockwise rotation
        elif pose.y >= 6.0 and self.previous_y_pose >= 6.0:
            self.previous_y_pose = pose.y
            self.Superman_pose = round(pose.x, 1)
            cmd.linear.x = 1.61
            cmd.angular.z = -abs(0.9) # for anticlockwise rotation
            if self.Cinderella_pose < 5.6 and self.Superman_pose > 5.3: #to make turtle to stop
                self.previous_y_pose = pose.y
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        elif pose.y < 6.0 and self.previous_y_pose < 6.0:
            self.previous_y_pose = pose.y
            cmd.linear.x = 2.0
            cmd.angular.z = 0.0

        self.cmd_vel_publisher_Superman.publish(cmd)
    

    def call_spawn_service(self, x, y, theta, name):
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for service...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn))

    def callback_spawn(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))


def main(args=None):
    rclpy.init()
    node = Turtle_Love_Node()
    rclpy.spin(node)
    rclpy.shutdown()
