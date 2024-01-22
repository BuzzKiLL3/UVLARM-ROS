#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import time
import math

#message to publish:
from geometry_msgs.msg import Twist

rosNode = None

def scan_callback(scanMsg):
    global rosNode
    angle = scanMsg.angle_min + math.pi/2
    obstacles = []
    cmd_debug_points_left = []
    cmd_debug_points_right = []

    for aDistance in scanMsg.ranges:
        if 0.15 < aDistance < 3.0:
            aPoint = [
                math.cos(angle) * aDistance,
                math.sin(angle) * aDistance,
                0.0
            ]
            obstacles.append(aPoint)
            if (0.05 < aPoint[0] < 0.02 and 0.3 < aPoint[1] < 0.7) or (0.01 < aPoint[0] < 0.1 and 0.1 < aPoint[1] < 0.3 ) :
                obstacles_right = True
                cmd_debug_points_right.append(aPoint)
            if (-0.05 < aPoint[0] < -0.02 and -0.3 < aPoint[1] < -0.7)or(-0.01 < aPoint[0] < -0.1 and -0.1 < aPoint[1] < -0.3 ):
                obstacles_left = True
                cmd_debug_points_left.append(aPoint)
        angle += scanMsg.angle_increment

    velo = Twist()

    
        
    if (len(cmd_debug_points_right) - len(cmd_debug_points_left)) > 10:
        print("go Left")
        velo.angular.z = 0.01 * (len(cmd_debug_points_right) + len(cmd_debug_points_left)) + 0.015 * (len(cmd_debug_points_right) - len(cmd_debug_points_left))
        velo.linear.x = 0.0
    
    elif (len(cmd_debug_points_left) - len(cmd_debug_points_right)) > 15:
        print("go right")
        velo.angular.z = 0.01 * (len(cmd_debug_points_right) + len(cmd_debug_points_left)) + 0.015 * (len(cmd_debug_points_right) - len(cmd_debug_points_left))
        velo.linear.x = 0.0

    else:
        speed = 0.3 - 0.05 *(len(cmd_debug_points_right) + len(cmd_debug_points_left))
        if speed < 0:
            speed = 0.0
        velo.linear.x = speed
        velo.angular.z = 0.01 * (len(cmd_debug_points_right) - len(cmd_debug_points_left))

    print(velo.linear.x)
    print(velo.angular.z)
        

    

    velocity_publisher.publish(velo)
    cloudPoints = pc2.create_cloud_xyz32(Header(frame_id='laser_link'),obstacles)
    cloud_publisher.publish(cloudPoints)


if __name__ == '__main__':
    print("move move move")
    rclpy.init()
    rosNode = Node('PC_Publisher')
    velocity_publisher = rosNode.create_publisher(Twist, '/multi/cmd_nav' , 10)
    cloud_publisher = rosNode.create_publisher(pc2.PointCloud2,'laser_link',10)
    rosNode.create_subscription( LaserScan, 'scan', scan_callback, 10)
    
    while True :
        rclpy.spin_once(rosNode , timeout_sec=0.1 )
    scanInterpret.destroy_node()
    rclpy.shutdown() 