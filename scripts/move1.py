#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from slam_toolbox import Trajectory
from nav_msgs.msg import Odometry
import math

rosNode = None
slam_trajectory_pub = None

def scan_callback(scanMsg):
    global rosNode, slam_trajectory_pub
    angle = scanMsg.angle_min + math.pi / 2
    obstacles = []
    cmd_debug_points_left = []
    cmd_debug_points_right = []

    for aDistance in scanMsg.ranges:
        if 0.15 < aDistance < 5.0:
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
        angle += scanMsg.angle_increment

    velo = Twist()

    left_distance = sum(p[0] for p in cmd_debug_points_left) / len(cmd_debug_points_left) if cmd_debug_points_left else 0.0
    right_distance = sum(p[0] for p in cmd_debug_points_right) / len(cmd_debug_points_right) if cmd_debug_points_right else 0.0

    if left_distance > right_distance:
        print("Turn left, more space on the left")
        velo.angular.z = 0.1
    elif right_distance > left_distance:
        print("Turn right, more space on the right")
        velo.angular.z = -0.1
    else:
        print("Move straight, equal space on both sides")
        velo.linear.x = 0.2

    velocity_publisher.publish(velo)

    # Publish SLAM trajectory
    trajectory_msg = Trajectory()
    # Set the trajectory based on your robot's pose
    # For simplicity, this example sets a trajectory with just one pose
    pose = Odometry()
    pose.header.stamp = rosNode.get_clock().now().to_msg()
    pose.header.frame_id = 'base_link'  # Change the frame_id accordingly
    pose.pose.pose.position.x = 0.0
    pose.pose.pose.position.y = 0.0
    pose.pose.pose.position.z = 0.0
    pose.pose.pose.orientation.w = 1.0
    trajectory_msg.poses.append(pose.pose.pose)
    slam_trajectory_pub.publish(trajectory_msg)

    # Publish point cloud
    cloudPoints = pc2.create_cloud_xyz32(Header(frame_id='laser_link'), obstacles)
    cloud_publisher.publish(cloudPoints)


if __name__ == '__main__':
    print("move move move")
    rclpy.init()
    rosNode = Node('PC_Publisher')
    velocity_publisher = rosNode.create_publisher(Twist, '/multi/cmd_nav', 10)
    cloud_publisher = rosNode.create_publisher(pc2.PointCloud2, 'laser_link', 10)
    slam_trajectory_pub = rosNode.create_publisher(Trajectory, 'trajectory', 1)
    rosNode.create_subscription(LaserScan, 'scan', scan_callback, 10)

    while rclpy.ok():
        rclpy.spin_once(rosNode, timeout_sec=0.1)
    rosNode.destroy_node()
    rclpy.shutdown()
