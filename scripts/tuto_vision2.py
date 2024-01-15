#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import pyrealsense2 as rs
import math
import signal

class RealSenseVisionNode(Node):
    def __init__(self):
        super().__init__('realsense_vision')

        self.bridge = CvBridge()

        # RealSense setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.colorizer = rs.colorizer()

        # FPS set to 30
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline.start(self.config)

        self.align_to = rs.stream.depth
        self.align = rs.align(self.align_to)

        self.publisher_color = self.create_publisher(Image, 'realsense/color', 10)
        self.publisher_depth = self.create_publisher(Image, 'realsense/depth', 10)
        self.publisher_object_detection = self.create_publisher(Image, 'realsense/object_detection', 10)

        # Object detection setup
        self.template = cv.imread('car.png')
        self.w, self.h = self.template.shape[::-1]
        self.threshold_template = 0.3
        self.lo = np.array([60 - 15, 50, 50])
        self.hi = np.array([60 + 6, 255, 255])
        self.kernel = np.ones((3, 3), np.uint8)

        # Flag to check if the template is detected
        self.object_detected = False

        # Distance threshold for object detection (increased distance)
        self.distance_threshold = 1.0  # Adjust as needed

        # Capture ctrl-c event
        self.is_ok = True
        signal.signal(signal.SIGINT, self.signal_interrupt)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def signal_interrupt(self, signum, frame):
        self.get_logger().info("\nCtrl-c pressed")
        self.is_ok = False

    def timer_callback(self):
        # Wait for a coherent tuple of frames: depth, color, and accel
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not aligned_color_frame:
            return

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(aligned_color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

        # Object detection
        hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_image, self.lo, self.hi)
        mask = cv.erode(mask, self.kernel, iterations=1)
        mask = cv.dilate(mask, self.kernel, iterations=1)
        image_segmented = cv.bitwise_and(color_image, color_image, mask=mask)

        img_gray_segmented = cv.cvtColor(image_segmented, cv.COLOR_BGR2GRAY)
        res = cv.matchTemplate(img_gray_segmented, self.template, cv.TM_CCOEFF_NORMED)
        loc = np.where(res >= self.threshold_template)

        # Reset the object_detected flag
        self.object_detected = False

        for pt in zip(*loc[::-1]):
            cv.rectangle(color_image, pt, (pt[0] + self.w, pt[1] + self.h), (0, 0, 255), 2)
            self.object_detected = True

        # Publish raw color image
        msg_color = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
        msg_color.header.stamp = self.get_clock().now().to_msg()
        msg_color.header.frame_id = 'color'
        self.publisher_color.publish(msg_color)

        # Publish raw depth image
        msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap, 'bgr8')
        msg_depth.header.stamp = msg_color.header.stamp
        msg_depth.header.frame_id = 'depth'
        self.publisher_depth.publish(msg_depth)

        # Publish color image with object detection
        msg_object_detection = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
        msg_object_detection.header.stamp = msg_color.header.stamp
        msg_object_detection.header.frame_id = 'object_detection'
        self.publisher_object_detection.publish(msg_object_detection)

class ScanInterpreterNode(Node):
    def __init__(self):
        super().__init__('scan_interpreter')
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

    def scan_callback(self, scan_msg):
        self.get_logger().info(f"scan:\n{scan_msg}")

def main(args=None):
    rclpy.init(args=args)

    vision_node = RealSenseVisionNode()
    scan_interpreter_node = ScanInterpreterNode()

    while rclpy.ok():
        rclpy.spin_once(vision_node)
        rclpy.spin_once(scan_interpreter_node)

    vision_node.destroy_node()
    scan_interpreter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
