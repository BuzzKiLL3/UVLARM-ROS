#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path  # Import Path message
import numpy as np
import pyrealsense2 as rs
import math
import cv2 as cv

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

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

        self.color_info = (0, 0, 255)
        self.rayon = 10

        # List of template files to compare
        self.template_files = ['template.png', 'car.png']

        # Dictionary to store threshold values for each template
        self.thresholds = {'template.png': 0.3, 'car.png': 0.5}

        # Flag to check if any template is detected
        self.object_detected = False

        # Creating morphological kernel
        self.kernel = np.ones((3, 3), np.uint8)

        # Create a dictionary to store templates
        self.templates = {}
        for template_file in self.template_files:
            self.templates[template_file] = cv.imread(template_file, 0)

        # Image publisher
        self.image_pub = self.create_publisher(Image, 'object_detection_image', 10)
        # Pose publisher
        self.pose_pub = self.create_publisher(PoseStamped, 'object_pose', 10)
        # Marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, 'object_marker', 10)
        # Path publisher
        self.path_pub = self.create_publisher(Path, 'robot_path', 10)
        self.robot_path = Path()
        self.visited_locations = set()

        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera_info', self.camera_info_callback, 10)

        # Initialize CV Bridge
        self.bridge = CvBridge()
        self.camera_info = None

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def convert_pixel_to_meter(self, pixel_x, pixel_y, depth):
        if self.camera_info is None:
            return None, None, None

        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy

        return x, y, depth

    def run(self):
        try:
            while rclpy.ok():
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
                if distance < 1.0:  # Adjust as needed
                    # Convert color image to HSV
                    hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)

                    # Segmentation in HSV color space
                    mask = cv.inRange(hsv_image, np.array([45, 50, 50]), np.array([75, 255, 255]))
                    mask = cv.erode(mask, self.kernel, iterations=1)
                    mask = cv.dilate(mask, self.kernel, iterations=1)
                    image_segmented = cv.bitwise_and(color_image, color_image, mask=mask)

                    # Loop over templates for comparison
                    for template_file, template in self.templates.items():
                        res = cv.matchTemplate(cv.cvtColor(image_segmented, cv.COLOR_BGR2GRAY), template,
                                               cv.TM_CCOEFF_NORMED)
                        loc = np.where(res >= self.thresholds[template_file])

                        # Update the object_detected flag
                        if len(loc[0]) > 0:
                            self.object_detected = True

                    # Print object detection status in real-time
                    if self.object_detected:
                        self.get_logger().info("Object detected!")

                        # Convert pixel coordinates to 3D coordinates in the camera frame
                        x, y, depth = self.convert_pixel_to_meter(x, y, depth)
                        if x is not None:
                            # Convert camera frame to base frame (adjust translation and rotation if needed)
                            x_base = x
                            y_base = y
                            z_base = depth

                            # Check if the location is not visited
                            if (x_base, y_base, z_base) not in self.visited_locations:
                                # Create PoseStamped message
                                pose_msg = PoseStamped()
                                pose_msg.header.stamp = self.get_clock().now().to_msg()
                                pose_msg.header.frame_id = "base_link"

                                pose_msg.pose.position = Point(x_base, y_base, z_base)
                                pose_msg.pose.orientation.w = 1.0  # Assuming no rotation

                                # Publish the detected object's pose
                                self.pose_pub.publish(pose_msg)

                                # Publish a Marker message for visualization in RViz2
                                marker_msg = MarkerArray()
                                marker_msg.header = pose_msg.header
                                marker_msg.ns = "object_marker"
                                marker_msg.id = 0
                                marker_msg.type = MarkerArray.SPHERE
                                marker_msg.action = MarkerArray.ADD
                                marker_msg.pose = pose_msg.pose
                                marker_msg.scale.x = 0.1  # Adjust as needed
                                marker_msg.scale.y = 0.1
                                marker_msg.scale.z = 0.1
                                marker_msg.color.a = 1.0
                                marker_msg.color.r = 1.0
                                marker_msg.color.g = 0.0
                                marker_msg.color.b = 0.0

                                self.marker_pub.publish(marker_msg)

                                # Add the new location to the robot's path
                                pose_msg.header.stamp = self.get_clock().now().to_msg()
                                self.robot_path.poses.append(pose_msg)

                                # Publish the robot's path
                                self.path_pub.publish(self.robot_path)

                                # Store visited locations to avoid revisiting
                                self.visited_locations.add((x_base, y_base, z_base))

                else:
                    self.get_logger().info("Object not detected.")

        except Exception as e:
            self.get_logger().error(str(e))

        finally:
            self.pipeline.stop()
            cv.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    object_detection_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
