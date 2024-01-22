#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import cv2 as cv
import numpy as np
import pyrealsense2 as rs
import math

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.bridge = CvBridge()
        self.marker_pub = self.create_publisher(Marker, 'object_marker', 10)
        self.image_sub = self.create_subscription(Image, 'image', self.image_callback, 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the frame to grayscale
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

        # Detect faces in the frame
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5, minSize=(30, 30))

        # Publish a marker if a face is detected
        if len(faces) > 0:
            x, y, w, h = faces[0]
            face_pose = Pose()
            face_pose.position.x = x + w/2
            face_pose.position.y = y + h/2
            face_pose.position.z = 0

            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = face_pose
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            self.marker_pub.publish(marker)
            self.get_logger().info("Object detected!")

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
