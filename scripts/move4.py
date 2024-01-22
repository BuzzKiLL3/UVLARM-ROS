#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import WheelDropEvent
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import time
import math

rosNode = None

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.leftWheelDropped = False
        self.rightWheelDropped = False
        self.cmd_vel_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(WheelDropEvent, 'event/wheel_drop', self.wheel_callback, 50)
        self.cmd_vel_msg = Twist()

    def wheel_callback(self, wheel_msg):
        # Check if either wheel is dropped
        if wheel_msg.state == WheelDropEvent.WHEEL_DROPPED:
            self.leftWheelDropped = True if wheel_msg.wheel == WheelDropEvent.WHEEL_LEFT else False
            self.rightWheelDropped = True if wheel_msg.wheel == WheelDropEvent.WHEEL_RIGHT else False

    def scan_callback(self, scan_msg):
        global rosNode
        angle = scan_msg.angle_min + math.pi / 2
        obstacles = []
        cmd_debug_points_left = []
        cmd_debug_points_right = []

        # Check if either wheel is dropped
        if self.leftWheelDropped or self.rightWheelDropped:
            velo = Twist()
            velo.linear.x = 0.0
            velo.angular.z = 0.0
            self.cmd_vel_publisher.publish(velo)
            return

        for aDistance in scan_msg.ranges:
            if 0.1 < aDistance < 3.0:
                aPoint = [
                    math.cos(angle) * aDistance,
                    math.sin(angle) * aDistance,
                    0.0
                ]
                obstacles.append(aPoint)
                if (0.01 < aPoint[0] < 0.2 and 0.3 < aPoint[1] < 0.7) or (0.01 < aPoint[0] < 0.1 and 0.1 < aPoint[1] < 0.3):
                    cmd_debug_points_right.append(aPoint)
                if (-0.2 < aPoint[0] < -0.01 and 0.3 < aPoint[1] < 0.7) or (-0.1 < aPoint[0] < -0.01 and 0.1 < aPoint[1] < 0.3):
                    cmd_debug_points_left.append(aPoint)
            angle += scan_msg.angle_increment

        velo = Twist()

        if (len(cmd_debug_points_right) - len(cmd_debug_points_left)) > 15:
            print("go Left")
            velo.angular.z = 0.03 * (len(cmd_debug_points_right) + len(cmd_debug_points_left)) + 0.015 * (
                    len(cmd_debug_points_right) - len(cmd_debug_points_left))
            velo.linear.x = 0.0

        elif (len(cmd_debug_points_left) - len(cmd_debug_points_right)) > 15:
            print("go right")
            velo.angular.z = 0.01 * (len(cmd_debug_points_right) + len(cmd_debug_points_left)) + 0.015 * (
                    len(cmd_debug_points_right) - len(cmd_debug_points_left))
            velo.linear.x = 0.0

        else:
            speed = 0.3 - 0.05 * (len(cmd_debug_points_right) + len(cmd_debug_points_left))
            if speed < 0:
                speed = 0.0
            velo.linear.x = speed
            velo.angular.z = 0.01 * (len(cmd_debug_points_right) - len(cmd_debug_points_left))

        print(velo.linear.x)
        print(velo.angular.z)

        self.cmd_vel_publisher.publish(velo)
        cloudPoints = pc2.create_cloud_xyz32(Header(frame_id='laser_link'), obstacles)
        cloud_publisher = rosNode.create_publisher(PointCloud2, 'laser_link', 10)
        cloud_publisher.publish(cloudPoints)

def main():
    global rosNode
    print("move move move")
    rclpy.init()
    rosNode = RobotController()
    
    try:
        rclpy.spin(rosNode)
    except KeyboardInterrupt:
        pass

    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
