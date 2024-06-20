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

    def joy_recived_command(self, msg):
         joy = msg
         ps = Joy()
         ps.buttons
         velocity = Twist()
         if joy.buttons[6] == 0:
            velocity.linear.x = joy.buttons[8]*1.0
            if joy.buttons[7] == 1:
                velocity.linear.x = joy.buttons[7]*-1.0
                velocity.angular.z = joy.axes[0]
            else:
                velocity.angular.z = joy.axes[0]

         else:
            velocity.linear.x = joy.axes[1]*joy.buttons[8]*4
            velocity.linear.z = float(joy.buttons[0])
            velocity.angular.z = joy.axes[0]*4
         self.joy_pub.publish(velocity)
         print(joy.buttons[6])



         
 



    

def main(args=None):
        rclpy.init(args=args)
        node = JoyNode()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()