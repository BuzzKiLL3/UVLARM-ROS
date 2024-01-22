#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import math

from kobuki_ros_interfaces.msg import WheelDropEvent

class ScanCallback(Node):
    def __init__(self):
        super().__init__('scan_callback_node')
        
        self.obstacles_left = False
        self.obstacles_right = False
        self.velocity_publisher = None
        self.cloud_publisher = None
        self.leftWheelDropped = False
        self.rightWheelDropped = False
        self.stopped = True
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
        angle = scan_msg.angle_min + math.pi/2
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

        # Add print statements for debugging
        print("Laser Scan Callback Triggered")

        for aDistance in scan_msg.ranges:
            if 0.1 < aDistance < 3.0:
                aPoint = [
                    math.cos(angle) * aDistance,
                    math.sin(angle) * aDistance,
                    0.0
                ]
                obstacles.append(aPoint)
                if (0.01 < aPoint[0] < 0.2 and 0.3 < aPoint[1] < 0.7) or (0.01 < aPoint[0] < 0.1 and 0.1 < aPoint[1] < 0.3):
                    self.obstacles_right = True
                    cmd_debug_points_right.append(aPoint)
                if (-0.2 < aPoint[0] < -0.01 and 0.3 < aPoint[1] < 0.7) or (-0.1 < aPoint[0] < -0.01 and 0.1 < aPoint[1] < 0.3):
                    self.obstacles_left = True
                    cmd_debug_points_left.append(aPoint)
            angle += scan_msg.angle_increment

        velo = Twist()

        if (len(cmd_debug_points_right) - len(cmd_debug_points_left)) > 15:
            print("go Left")
            velo.angular.z = 0.03 * (len(cmd_debug_points_right) + len(cmd_debug_points_left)) + 0.015 * (len(cmd_debug_points_right) - len(cmd_debug_points_left))
            velo.linear.x = 0.0
    
        elif (len(cmd_debug_points_left) - len(cmd_debug_points_right)) > 15:
            print("go right")
            velo.angular.z = 0.01 * (len(cmd_debug_points_right) + len(cmd_debug_points_left)) + 0.015 * (len(cmd_debug_points_right) - len(cmd_debug_points_left))
            velo.linear.x = 0.0

        else:
            speed = 0.3 - 0.05 * (len(cmd_debug_points_right) + len(cmd_debug_points_left))
            if speed < 0:
                speed = 0.0
            velo.linear.x = speed
            velo.angular.z = 0.01 * (len(cmd_debug_points_right) - len(cmd_debug_points_left))

        print("Linear Velocity:", velo.linear.x)
        print("Angular Velocity:", velo.angular.z)

        self.cmd_vel_publisher.publish(velo)
        cloudPoints = pc2.create_cloud_xyz32(Header(frame_id='laser_link'), obstacles)
        self.cloud_publisher.publish(cloudPoints)


if __name__ == '__main__':
    print("move move move")
    rclpy.init()
    rosNode = Node('PC_Publisher')
    scan_callback_node = ScanCallback()
    scan_callback_node.velocity_publisher = rosNode.create_publisher(Twist, '/multi/cmd_nav', 10)
    scan_callback_node.cloud_publisher = rosNode.create_publisher(pc2.PointCloud2, 'laser_link', 10)
    
    try:
        while rclpy.ok():
            # Add a print statement for debugging
            print("Spinning Once")
            rclpy.spin_once(rosNode, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    rosNode.destroy_node()
    rclpy.shutdown()