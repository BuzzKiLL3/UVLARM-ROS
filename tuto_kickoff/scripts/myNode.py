#!/usr/bin/python3
import rclpy                 # core ROS2 client python library
from rclpy.node import Node  # To manipulate ROS Nodes

def main():
    rclpy.init()     # Initialize ROS2 client
    myNode= Node('blanc_node') # Create a Node, with a name         

    # Start the ros infinite loop with myNode.
    rclpy.spin( myNode )

    # At the end, destroy the node explicitly.
    myNode.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

    print("tuto_move :: STOP.")

# If the file is executed as a script (ie. not imported).
if __name__ == '__main__':
    # call main() function
    main()


