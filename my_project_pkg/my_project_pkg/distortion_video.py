#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
#from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

from camera_calibrate_params import camera_matrix, distortion_coefficients
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy


import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Define the QoS profile to match the expected QoS settings of the publisher


class OpticalFlowVelNode(Node):
    def __init__(self):
        super().__init__('undistorted_video')
        #self.subscriber_= self.create_subscription(Image,'undistort_image',self.callback_image, 10)
        self.fisheye_subscriber= self.create_subscription(CompressedImage,'/fisheye_cam/image_raw/compressed',self.callback_image, 10)


        
        # self.cam_sub = Subscriber(self, CompressedImage, '/fisheye_cam/image_raw/compressed')
        # self.gyro_subscriber = Subscriber(self, Imu, '/imu/data' )
        # self.sync = ApproximateTimeSynchronizer([self.cam_sub, self.gyro_subscriber], 10, 0.5, allow_headerless=True)
        
      
       

        self.contour_points = []
        self.cv_bridge = CvBridge()

       
        self.feature_params = dict(maxCorners = 100,
                      qualityLevel = 0.1,
                      minDistance = 7,
                      blockSize = 7)

        self.lk_params = dict(winSize = (15, 15),
                 maxLevel =2,
                 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        
        self.xfocal = camera_matrix[0, 0]
        self.yfocal = camera_matrix[1, 1]
        self.Cx = camera_matrix[2, 2]
        self.Cy = camera_matrix[1, 2]
        self.z = 1300
        self.frame_count = 0
        self.param_factor = 50/30.24727637
        self.Flag = 0
        self.contour = []
        self.p0 = None
        self.prev_gray = None
        self.point = []
        self.ave_dist_list =np.array([])
        self.counter = 0
        self.ave_vel = 0.0
        self.contour_counter = 0
        self.elapsed_time = None
        self.angle = 0
        self.deviation_list = []
        self.temp_deviation = 0
        self.previous_timestamp = None
        self.prev_time = None
        self.delta_t = 0
        self.distance = 0
        self.measure_theta = 0
        self.is_twist = 0



    def callback_image(self, image_msg):
       self.frame = self.cv_bridge.compressed_imgmsg_to_cv2(image_msg,'bgr8')
       

       h, w = self.frame.shape[:2]
       newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
            

            # crop the image
       x, y, w, h = roi
       dst = cv2.undistort(self.frame, camera_matrix, distortion_coefficients, None, newCameraMatrix)
            
       dst = dst[y:y + h, x:x + w]
       cv2.imshow('Fisheye_Camera', self.frame)
       cv2.imshow('undistortion_Camera', dst)

       cv2.waitKey(1)

       #self.get_logger().info('Publishing proccess image...')
     

def main(args=None):
    rclpy.init(args=args)
    optical_flow_node = OpticalFlowVelNode()
    while rclpy.ok():
        rclpy.spin_once(optical_flow_node, timeout_sec=0.1)  # Use spin_once with a timeout
    #rclpy.spin(optical_flow_node)
    optical_flow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    