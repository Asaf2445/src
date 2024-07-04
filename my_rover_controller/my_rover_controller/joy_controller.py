#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
import cv2
import numpy as np
from sensor_msgs.msg import Joy


class JoyNode(Node):

    def __init__(self):
        super().__init__("joy_controller")
        self.joy_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.arm_pub = self.create_publisher(JointTrajectory, "set_joint_trajectory", 10)
        #self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Node been started")
        self.joy_subscriber_= self.create_subscription(Joy, "joy", self.joy_recived_command, 10)
        self.theta=0.0

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

         elif joy.buttons[6] ==1 and joy.buttons[8] ==1 or joy.buttons[7] == 1:
            velocity.linear.x = joy.axes[1]*joy.buttons[8]*4
            velocity.linear.z = float(joy.buttons[0])
            velocity.angular.z = joy.axes[0]*4
         self.joy_pub.publish(velocity)
         print(joy.axes[3])

         joint_trajectory = JointTrajectory()

         joint_trajectory.header.frame_id = "base_footprint"
         joint_trajectory.joint_names = ["arm_base_forearm_joint", "forearm_hand_joint"]

         point = JointTrajectoryPoint()
         point.time_from_start.sec = 1  # time from the start of the trajectory

        # Define angular velocities for each joint
        #  point.velocities = [joy.axes[3], joy.axes[3]]  # Angular velocities in radians per second for each joint
        # You can also set positions, accelerations, and effort if needed
         self.theta += joy.axes[3]*0.001
         point.positions = [0.0, self.theta]  # Initial positions for the joints (optional)
        #  point.accelerations = [0.0, 0.0]  # Initial accelerations for the joints (optional)
        #  point.effort = [0.0, 0.0]  # Initial effort for the joints (optional)

         joint_trajectory.points = [point]

         self.arm_pub.publish(joint_trajectory)


def main(args=None):
        rclpy.init(args=args)
        node = JoyNode()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()