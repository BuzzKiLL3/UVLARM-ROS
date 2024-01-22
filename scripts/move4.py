#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from kobuki_ros_interfaces.msg import WheelDropEvent

class LiftDetectionNode(Node):
    def __init__(self):
        super().__init__('lift_detection_node')
        self.left_wheel_dropped = False
        self.right_wheel_dropped = False
        self.create_subscription(WheelDropEvent, 'event/wheel_drop', self.wheel_callback, 10)
        self.create_timer(1.0, self.print_lift_status)  # Print status every 1 second

    def wheel_callback(self, wheel_msg):
        # Check if left wheel is dropped
        self.left_wheel_dropped = wheel_msg.wheel == WheelDropEvent.WHEEL_LEFT

        # Check if right wheel is dropped
        self.right_wheel_dropped = wheel_msg.wheel == WheelDropEvent.WHEEL_RIGHT

        # Debugging: Print received message details
        self.get_logger().info("Received WheelDropEvent: %s, Left Dropped: %s, Right Dropped: %s",
                               wheel_msg.wheel, self.left_wheel_dropped, self.right_wheel_dropped)

    def print_lift_status(self):
        if self.left_wheel_dropped and self.right_wheel_dropped:
            print("Robot is lifted!")
        elif not self.left_wheel_dropped and not self.right_wheel_dropped:
            print("Both wheels are on the floor.")
        else:
            print("One wheel is on the floor, and the other is lifted.")

def main():
    print("Lift Detection Node")
    rclpy.init()
    node = LiftDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
