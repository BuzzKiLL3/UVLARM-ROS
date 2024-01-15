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

    def scan_callback(self, msg):
        # Process laser scan data for obstacle detection
        obstacle_detected = any(distance < 1.0 for distance in msg.ranges)
        
        # Randomly move the robot
        velo = Twist()
        if not obstacle_detected:
            velo.linear.x = random.uniform(0.1, 0.5)  # Random linear velocity between 0.1 and 0.5 m/s
            velo.angular.z = random.uniform(-0.5, 0.5)  # Random angular velocity between -0.5 and 0.5 rad/s
        self.velocity_publisher.publish(velo)

def main():
    rclpy.init()
    node = RandomMoveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
