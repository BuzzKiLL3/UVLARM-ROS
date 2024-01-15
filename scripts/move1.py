#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import math

rosNode = None

def calculate_sensitivity_factor(distance):
    # Define a function to calculate the sensitivity factor based on distance
    # You can customize this function to meet your sensitivity needs
    max_distance = 5.0  # Adjust this based on the maximum expected distance
    sensitivity_factor = 3.0 - min(distance / max_distance, 1.0)
    return sensitivity_factor

def scan_callback(scanMsg):
    global rosNode
    angle = scanMsg.angle_min + math.pi / 2
    obstacles_left = 0
    obstacles_right = 0
    obstacles = []
    cmd_debug_points_left = []
    cmd_debug_points_right = []

    for aDistance in scanMsg.ranges:
        if 0.1 < aDistance < 5.0:
            aPoint = [
                math.cos(angle) * aDistance,
                math.sin(angle) * aDistance,
                0.0
            ]
            obstacles.append(aPoint)
            if (0.01 < aPoint[0] < 0.2 and 0.3 < aPoint[1] < 0.7) or (0.01 < aPoint[0] < 0.1 and 0.1 < aPoint[1] < 0.3):
                obstacles_right += 1
                cmd_debug_points_right.append(aPoint)
            if (-0.2 < aPoint[0] < -0.01 and 0.3 < aPoint[1] < 0.7) or (-0.1 < aPoint[0] < -0.01 and 0.1 < aPoint[1] < 0.3):
                obstacles_left += 1
                cmd_debug_points_left.append(aPoint)
        angle += scanMsg.angle_increment  # Adjust this increment for a smaller angle

    velo = Twist()

    sensitivity_factor = calculate_sensitivity_factor(scanMsg.ranges[0])  # Using the first distance as a reference
    
    if obstacles_right > obstacles_left:
        print("Go Left")
        velo.angular.z = 0.3 * sensitivity_factor
        velo.linear.x = 0.0
    elif obstacles_left > obstacles_right:
        print("Go Right")
        velo.angular.z = -0.3 * sensitivity_factor
        velo.linear.x = 0.0
    else:
        print("Go Forward")
        velo.angular.z = 0.0
        velo.linear.x = 0.3 * sensitivity_factor

    velocity_publisher.publish(velo)
    cloudPoints = pc2.create_cloud_xyz32(Header(frame_id='laser_link'), obstacles)
    cloud_publisher.publish(cloudPoints)

if __name__ == '__main__':
    print("move move move")
    rclpy.init()
    rosNode = Node('PC_Publisher')
    velocity_publisher = rosNode.create_publisher(Twist, '/cmd_vel', 10)
    cloud_publisher = rosNode.create_publisher(pc2.PointCloud2, 'laser_link', 10)
    
    # Adjust the angle increment for a smaller angle
    scan_subscription = rosNode.create_subscription(LaserScan, 'scan', scan_callback, 10)
    scan_subscription.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    while True:
        rclpy.spin_once(rosNode, timeout_sec=0.1)
    rosNode.destroy_node()
    rclpy.shutdown()
