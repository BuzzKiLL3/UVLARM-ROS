#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np
import pyrealsense2 as rs
import math
import cv2 as cv

class ObjectDetectionNode(Node):
    COLOR_INFO = (0, 0, 255)
    RAYON = 10
    TEMPLATE_FILES = ['template.png', 'car.png']
    THRESHOLDS = {'template.png': 0.3, 'car.png': 0.5}
    SEGMENTATION_LOWER = np.array([45, 50, 50])
    SEGMENTATION_UPPER = np.array([75, 255, 255])
    DISTANCE_THRESHOLD = 1.0

    def __init__(self):
        super().__init__('object_detection_node')

        # RealSense setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.colorizer = rs.colorizer()

        # FPS set to 15 (reduced for demo purposes)
        self.config.enable_stream(rs.stream.depth, 440, 280, rs.format.z16, 15)
        self.config.enable_stream(rs.stream.color, 440, 280, rs.format.bgr8, 15)
        self.config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 30)
        self.config.enable_stream(rs.stream.infrared, 2, 848, 480, rs.format.y8, 30)

        self.pipeline.start(self.config)

        self.align_to = rs.stream.depth
        self.align = rs.align(self.align_to)

        # Other setup
        self.color_info = self.COLOR_INFO
        self.rayon = self.RAYON
        self.object_detected = False
        self.kernel = np.ones((3, 3), np.uint8)
        self.templates = {file: cv.imread(file, 0) for file in self.TEMPLATE_FILES}
        self.bridge = CvBridge()
        self.camera_info = None

        # Image publisher for raw color image
        self.raw_image_pub = self.create_publisher(Image, 'raw_color_image', 10)
        # Pose publisher
        self.pose_pub = self.create_publisher(PoseStamped, 'object_pose', 10)
        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, 'object_marker', 10)

        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera_info', self.camera_info_callback, 10)

        # Image publisher for RViz2
        self.image_publisher = self.create_publisher(Image, 'image', 10)
        # Image publishers for infrared images
        self.infra_publisher_1 = self.create_publisher(Image, 'infrared_1', 10)
        self.infra_publisher_2 = self.create_publisher(Image, 'infrared_2', 10)

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
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                aligned_color_frame = aligned_frames.get_color_frame()
                infra_frame_1 = frames.get_infrared_frame(1)
                infra_frame_2 = frames.get_infrared_frame(2)

                if not depth_frame or not aligned_color_frame or not infra_frame_1 or not infra_frame_2:
                    continue

                colorized_depth = self.colorizer.colorize(depth_frame)
                depth_colormap = np.asanyarray(colorized_depth.get_data())

                color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
                color_image = np.asanyarray(aligned_color_frame.get_data())

                # Publish raw color image to 'raw_color_image' topic
                raw_color_image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                self.raw_image_pub.publish(raw_color_image_msg)

                # Publish color image to 'image' topic for RViz2 visualization
                msg_image = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
                msg_image.header.stamp = self.get_clock().now().to_msg()
                msg_image.header.frame_id = "camera_link"
                self.image_publisher.publish(msg_image)

                # Publish infrared images to 'infrared_1' and 'infrared_2' topics
                infra_image_1 = np.asanyarray(infra_frame_1.get_data())
                infra_image_2 = np.asanyarray(infra_frame_2.get_data())
                infra_colormap_1 = cv.applyColorMap(cv.convertScaleAbs(infra_image_1, alpha=0.03), cv.COLORMAP_JET)
                infra_colormap_2 = cv.applyColorMap(cv.convertScaleAbs(infra_image_2, alpha=0.03), cv.COLORMAP_JET)

                msg_infra_1 = self.bridge.cv2_to_imgmsg(infra_colormap_1, "bgr8")
                msg_infra_1.header.stamp = msg_image.header.stamp
                msg_infra_1.header.frame_id = "camera_link"
                self.infra_publisher_1.publish(msg_infra_1)

                msg_infra_2 = self.bridge.cv2_to_imgmsg(infra_colormap_2, "bgr8")
                msg_infra_2.header.stamp = msg_image.header.stamp
                msg_infra_2.header.frame_id = "camera_link"
                self.infra_publisher_2.publish(msg_infra_2)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape
                x, y = int(color_colormap_dim[1] / 2), int(color_colormap_dim[0] / 2)
                depth = depth_frame.get_distance(x, y)
                dx, dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x, y], depth)
                distance = math.sqrt(dx**2 + dy**2 + dz**2)

                if distance < self.DISTANCE_THRESHOLD:
                    hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)
                    mask = cv.inRange(hsv_image, self.SEGMENTATION_LOWER, self.SEGMENTATION_UPPER)
                    mask = cv.erode(mask, self.kernel, iterations=1)
                    mask = cv.dilate(mask, self.kernel, iterations=1)
                    image_segmented = cv.bitwise_and(color_image, color_image, mask=mask)

                    for template_file, template in self.templates.items():
                        res = cv.matchTemplate(cv.cvtColor(image_segmented, cv.COLOR_BGR2GRAY), template,
                                               cv.TM_CCOEFF_NORMED)
                        loc = np.where(res >= self.THRESHOLDS[template_file])

                        if len(loc[0]) > 0:
                            self.object_detected = True
                            break

                    if self.object_detected:
                        self.get_logger().info("Object detected!")

                        x, y, depth = self.convert_pixel_to_meter(x, y, depth)
                        if x is not None:
                            x_base, y_base, z_base = x, y, depth

                            if (x_base, y_base, z_base) not in self.visited_locations:
                                pose_msg = PoseStamped()
                                pose_msg.header.stamp = self.get_clock().now().to_msg()
                                pose_msg.header.frame_id = "base_link"
                                pose_msg.pose.position = Point(x_base, y_base, z_base)
                                pose_msg.pose.orientation.w = 1.0

                                self.pose_pub.publish(pose_msg)

                                marker_msg = Marker()
                                marker_msg.header = pose_msg.header
                                marker_msg.ns = "object_marker"
                                marker_msg.id = 0  # Same ID for each detection
                                marker_msg.type = Marker.SPHERE
                                marker_msg.action = Marker.ADD
                                marker_msg.pose = pose_msg.pose
                                marker_msg.scale.x = 0.1
                                marker_msg.scale.y = 0.1
                                marker_msg.scale.z = 0.1
                                marker_msg.color.a = 1.0
                                marker_msg.color.r = 0.0
                                marker_msg.color.g = 0.0
                                marker_msg.color.b = 1.0  # Blue color

                                self.marker_pub.publish(marker_msg)

                                self.visited_locations.add((x_base, y_base, z_base))
                                # Reset the flag after publishing the marker
                                self.object_detected = False

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
