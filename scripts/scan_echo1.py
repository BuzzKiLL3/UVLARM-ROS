#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Point32
import math

class ScanInterpreterNode(Node):
    def __init__(self):
        super().__init__('scan_interpreter')
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

    def scan_callback(self, scanMsg):
        obstacles = []
        angle = scanMsg.angle_min

        for aDistance in scanMsg.ranges:
            if 0.1 < aDistance and aDistance < 5.0:
                aPoint = [
                    math.cos(angle) * aDistance,
                    math.sin(angle) * aDistance
                ]
                obstacles.append(aPoint)
            angle += scanMsg.angle_increment

        sample = [[round(p[0], 2), round(p[1], 2)] for p in obstacles[10:20]]
        self.get_logger().info(f"obs({len(obstacles)}) ...{sample}...")

        # Assuming you want to publish Point32 messages for each obstacle
        for obstacle_point in obstacles:
            point_msg = Point32()
            point_msg.x = float(obstacle_point[0])
            point_msg.y = float(obstacle_point[1])
            point_msg.z = float(0)
            # Publish the Point32 message here

def main():
    rclpy.init()
    node = ScanInterpreterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
