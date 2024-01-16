#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import math

class ScanInterpreterNode(Node):
    def __init__(self):
        super().__init__('scan_interpreter')

        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'pointcloud', 10)

    def scan_callback(self, scan_msg):
        self.get_logger().info(f"Received LaserScan:\n{scan_msg}")

        # Publish LaserScan data
        self.scan_publisher.publish(scan_msg)

        # Convert LaserScan to PointCloud2
        pointcloud_msg = self.convert_scan_to_pointcloud2(scan_msg)
        self.pointcloud_publisher.publish(pointcloud_msg)

    def convert_scan_to_pointcloud2(self, scan_msg):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = scan_msg.header.frame_id

        points = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if not math.isinf(r) and not math.isnan(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0  # Assuming a planar surface

                point = [x, y, z]
                points.append(point)

            angle += scan_msg.angle_increment

        fields = pc2.create_cloud_xyz32().fields
        pointcloud_msg = pc2.create_cloud_xyz32(header, fields, points)

        return pointcloud_msg

def main():
    rclpy.init()
    scan_interpreter_node = ScanInterpreterNode()

    # Subscribe to LaserScan and publish PointCloud2
    scan_interpreter_node.create_subscription(LaserScan, 'scan', scan_interpreter_node.scan_callback, 10)

    rclpy.spin(scan_interpreter_node)

    scan_interpreter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
