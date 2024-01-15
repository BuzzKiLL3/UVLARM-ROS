#!/usr/bin/python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

rosNode= None

def scan_callback(scanMsg):
    global rosNode
    rosNode.get_logger().info( f"scan:\n{scanMsg}" )
    obstacles= []
    angle= scanMsg.angle_min
    for aDistance in scanMsg.ranges :
        if 0.1 < aDistance and aDistance < 5.0 :
            aPoint= [   
            math.cos(angle) * aDistance,
            math.sin(angle) * aDistance
        ]
        obstacles.append(aPoint)
    angle+= scanMsg.angle_increment

rclpy.init()
rosNode= Node('scan_interpreter')
rosNode.create_subscription( LaserScan, 'scan', scan_callback, 10)

while True :
    rclpy.spin_once( rosNode )
scanInterpret.destroy_node()
rclpy.shutdown()


def main():
    print('Move move move !')

if __name__ == '_main__' :
    main()