#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import pyrealsense2 as rs
import math
from rclpy import  init
from rclpy import  Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'image', 10)
        self.detection_pub = self.create_publisher(bool, 'object_detection', 10)

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

        # Template matching setup
        self.template = cv.imread('template.png', 0)
        self.w, self.h = self.template.shape[::-1]
        self.threshold_template = 0.3

        # Segmentation parameters using HSV color space
        self.color = 60
        self.lo = np.array([self.color - 15, 50, 50])
        self.hi = np.array([self.color + 6, 255, 255])

        # Flag to check if the template is detected
        self.object_detected = False

        # Distance threshold for object detection (increased distance)
        self.distance_threshold = 1.0  # Adjust as needed

        # Creating morphological kernel
        self.kernel = np.ones((3, 3), np.uint8)

    def image_callback(self):
        try:
            while True:
                # RealSense frames
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                aligned_color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not aligned_color_frame:
                    continue

                # Colorized depth map
                colorized_depth = self.colorizer.colorize(depth_frame)
                depth_colormap = np.asanyarray(colorized_depth.get_data())

                # Set background to black
                depth_colormap[depth_colormap == 0] = 0

                color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
                color_image = np.asanyarray(aligned_color_frame.get_data())

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                # Use pixel value of depth-aligned color image to get 3D axes
                x, y = int(color_colormap_dim[1] / 2), int(color_colormap_dim[0] / 2)
                depth = depth_frame.get_distance(x, y)
                dx, dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x, y], depth)
                distance = math.sqrt(((dx) ** 2) + ((dy) ** 2) + ((dz) ** 2))

                # Check if the object is within the specified distance
                if distance < self.distance_threshold:
                    # Convert color image to HSV
                    hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)

                    # Segmentation in HSV color space
                    mask = cv.inRange(hsv_image, self.lo, self.hi)
                    mask = cv.erode(mask, self.kernel, iterations=1)
                    mask = cv.dilate(mask, self.kernel, iterations=1)
                    image_segmented = cv.bitwise_and(color_image, color_image, mask=mask)

                    # Template matching with segmented image
                    img_gray_segmented = cv.cvtColor(image_segmented, cv.COLOR_BGR2GRAY)
                    res = cv.matchTemplate(img_gray_segmented, self.template, cv.TM_CCOEFF_NORMED)
                    loc = np.where(res >= self.threshold_template)

                    # Reset the object_detected flag
                    self.object_detected = False

                    for pt in zip(*loc[::-1]):
                        cv.rectangle(color_image, pt, (pt[0] + self.w, pt[1] + self.h), (0, 0, 255), 2)
                        self.object_detected = True

                    # Show images
                    images = np.hstack((color_image, depth_colormap, image_segmented))

                    cv.circle(images, (int(x), int(y)), int(self.rayon), self.color_info, 2)
                    cv.circle(images, (int(x + color_colormap_dim[1]), int(y)), int(self.rayon), self.color_info, 2)

                    cv.putText(images, "D=" + str(round(distance, 2)), (int(x) + 10, int(y) - 10),
                                cv.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv.LINE_AA)
                    cv.putText(images, "D=" + str(round(distance, 2)), (int(x + color_colormap_dim[1]) + 10, int(y) - 10),
                                cv.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv.LINE_AA)

                    # Show images
                    cv.imshow('RealSense', images)
                    cv.waitKey(0)

                    # Publish image
                    image_msg = self.bridge.cv2_to_imgmsg(images, encoding='bgr8')
                    self.image_pub.publish(image_msg)

                    # Print object detection status in real-time
                    detection_msg = bool()
                    detection_msg = self.object_detected
                    self.detection_pub.publish(detection_msg)

                    if self.object_detected:
                        print("Object detected!")
                    else:
                        print("Object not detected.")

        except Exception as e:
            print(e)

        finally:
            self.pipeline.stop()


if __name__ == '__main__':
    init()
    node = ObjectDetectionNode()
    node.image_callback()
