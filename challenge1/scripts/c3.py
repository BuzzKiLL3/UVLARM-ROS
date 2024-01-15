#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class RandomMoveNode(Node):
    def __init__(self):
        super().__init__('random_move_node')

        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.obstacle_distance_threshold = 1.0
        self.random_move_command = Twist()

    def scan_callback(self, msg):
        # Process laser scan data for obstacle detection
        obstacle_detected = any(distance < self.obstacle_distance_threshold for distance in msg.ranges)
        
        # Randomly change the robot's direction
        if not obstacle_detected:
            self.random_move_command.linear.x = random.uniform(0.1, 0.5)  # Random linear velocity between 0.1 and 0.5 m/s
            self.random_move_command.angular.z = random.uniform(-1.0, 1.0)  # Random angular velocity between -1.0 and 1.0 rad/s
        else:
            # If obstacle detected, stop and randomize direction
            self.random_move_command.linear.x = 0.0
            self.random_move_command.angular.z = random.uniform(-1.0, 1.0)

        # Publish the velocity command
        self.velocity_publisher.publish(self.random_move_command)

def main():
    rclpy.init()
    node = RandomMoveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
