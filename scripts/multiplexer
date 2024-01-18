#!/usr/bin/python3
import time, rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)

    # Initialize ScanInterperter
    node = Multiplexer()
    
    # infinite Loop
    rclpy.spin(node)
    
    # Clean end
    node.destroy_node()
    rclpy.shutdown()

class Multiplexer(Node):

    def __init__( self, MsgType= Twist, inTopics=['/multi/cmd_nav', '/multi/cmd_teleop'], outTopic='/cmd_vel' ):
        super().__init__('multiplexer')
        callbacks= [
            self.cmd_1_callback, self.cmd_2_callback, self.cmd_3_callback, self.cmd_4_callback,
            self.cmd_5_callback, self.cmd_6_callback, self.cmd_7_callback, self.cmd_8_callback
        ]
        callbacks= callbacks[:len(inTopics)]
        for topic, callback in zip(inTopics, callbacks) :
            self.create_subscription( MsgType, topic, callback, 10)
 
        self.hand= 0
        self.t= time.time()

        self.myPublisher= self.create_publisher( MsgType, outTopic, 10)

    def activateSecurity(self):
        self.create_timer(0.1, self.cmd_0)

    # CallBack gestion:
    def cmd_callback(self, prevalence, cmdMsg):
        t= time.time()
        if prevalence >= self.hand or t - self.t > 1.0 :
            if prevalence > self.hand :
                self.get_logger().info( f"Get cmd from {prevalence}" )
            self.hand= prevalence
            self.t= t
            self.myPublisher.publish( cmdMsg )

    def cmd_0(self):
        velo= Twist()
        return self.cmd_callback(0, velo)

    def cmd_1_callback(self, cmdMsg):
        return self.cmd_callback(1, cmdMsg)
    
    def cmd_2_callback(self, cmdMsg):
        return self.cmd_callback(2, cmdMsg)

    def cmd_3_callback(self, cmdMsg):
        return self.cmd_callback(3, cmdMsg)

    def cmd_4_callback(self, cmdMsg):
        return self.cmd_callback(4, cmdMsg)

    def cmd_5_callback(self, cmdMsg):
        return self.cmd_callback(5, cmdMsg)

    def cmd_6_callback(self, cmdMsg):
        return self.cmd_callback(6, cmdMsg)

    def cmd_7_callback(self, cmdMsg):
        return self.cmd_callback(7, cmdMsg)

    def cmd_8_callback(self, cmdMsg):
        return self.cmd_callback(8, cmdMsg)

if __name__ == "__main__" :
    main()