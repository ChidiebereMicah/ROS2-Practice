#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class MoveToPlaceNode(Node):
    
    def __init__(self):
        super().__init__("move_to_place")
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10 )
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("The turtle will now move to a place")
        #the turtles arfe to navigate to the point x:8 and y:8 in the turtlesim simulator
        self.target_x = 8.0
        self.target_y = 8.0

    '''
    #Trial 1
    def pose_callback(self, pose:Pose):
        cmd = Twist()
        self.get_logger().info(str(pose.x) + ' ' + str(pose.theta))

        #To move the turtle along the x-axis
        if pose.x <= 9.0:
            while pose.theta != 0:
                cmd.angular.z = 0.5
            while pose.x < 9.0:
                cmd.linear.x = 1.0
        else:
            while pose.theta != 22/7:
                cmd.angular.z = 0.5
            while pose.x > 9.0:
                cmd.linear.x = 1.0

        #To move the turtle along the y-axis 
        if pose.y <= 9.0:
            while pose.theta != 22/14:
                cmd.angular.z = 0.5
            while pose.y < 9.0:
                cmd.linear.x = 0.7
        else:
            while pose.theta != 33/7:
                cmd.angular.z = 0.5
            while pose.y > 9.0: 
                cmd.linear.x = 1.0
        #cmd.angular.z = 0.5

        self.cmd_vel_publisher.publish(cmd)


    #Trial 2
    def pose_callback(self, pose: Pose):
        cmd = Twist()
        self.get_logger().info(f'Current position: x={pose.x}, theta={pose.theta}')
        
        # Rotate to face the correct direction for x movement
        if pose.x < self.target_x:
            if abs(pose.theta) > 0.1:  # Small tolerance to avoid infinite loop
                cmd.angular.z = 0.01
        elif pose.x > self.target_x:
            if abs(pose.theta - math.pi) > 0.1:  # Small tolerance for 180 degrees
                cmd.angular.z = 0.01

        # Move along x-axis
        if pose.x < self.target_x:
            cmd.linear.x = 0.05
        elif pose.x > self.target_x:
            cmd.linear.x = 0.05
        
        # Rotate to face the correct direction for y movement
        if pose.y < self.target_y:
            if abs(pose.theta - math.pi/2) > 0.1:  # Small tolerance for 90 degrees
                cmd.angular.z = 0.01
        elif pose.y > self.target_y:
            if abs(pose.theta - 3*math.pi/2) > 0.1:  # Small tolerance for 270 degrees
                cmd.angular.z = 0.01

        # Move along y-axis
        if pose.y < self.target_y:
            cmd.linear.x = 0.05
        elif pose.y > self.target_y:
            cmd.linear.x = 0.05
        
        self.cmd_vel_publisher.publish(cmd)'''

    def pose_callback(self, pose: Pose):
        cmd = Twist()
        self.get_logger().info(f'Current position: x={pose.x}, theta={pose.theta}')
        
        # Rotate to face the correct direction for x movement
        if pose.x < self.target_x: # when turtle is to the left of target
            if abs(pose.theta) > 0.1:  # theta here should be 0 degrees but add
                                       # a small tolerance to avoid infinite loop
                cmd.angular.z = 0.1
            else:
                cmd.linear.x = 0.3
        elif pose.x > self.target_x: # when turtle is to the right of target
            if abs(pose.theta - math.pi) > 0.1:  # Small tolerance for 180 degrees
                cmd.angular.z = 0.1
            else:
                cmd.linear.x = 0.3

        # Rotate to face the correct direction for y movement
        if pose.y < self.target_y: # when turtle is below target
            if abs(pose.theta - math.pi/2) > 0.1:  # Small tolerance for 90 degrees
                cmd.angular.z = 0.1
            else:
                cmd.linear.x = 0.3
        elif pose.y > self.target_y: # when turtle is above target
            if abs(pose.theta - 3*math.pi/2) > 0.1:  # Small tolerance for 270 degrees
                cmd.angular.z = 0.1
            else:
                cmd.linear.x = 0.3
        
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPlaceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()