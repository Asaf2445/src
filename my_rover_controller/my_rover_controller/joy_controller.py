#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import random
import cv2
import numpy as np
from sensor_msgs.msg import Joy


class JoyNode(Node):

    def __init__(self):
        super().__init__("joy_controller")
        self.joy_pub = self.create_publisher(Twist, "cmd_vel", 10)
        #self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Node been started")
        self.joy_subscriber_= self.create_subscription(Joy, "joy", self.joy_recived_command, 10)

    def timer_callback(self):
         vel = Twist()
         vel.linear.x = 1.0
         self.joy_pub.publish(vel)


    def joy_recived_command(self, msg):
         joy = msg
         ps = Joy()
         velocity = Twist()
         velocity.linear.x = joy.axes[1]
         self.joy_pub.publish(velocity)



         
 



    

def main(args=None):
        rclpy.init(args=args)
        node = JoyNode()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()