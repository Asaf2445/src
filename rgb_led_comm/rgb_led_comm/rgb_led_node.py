#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
import cv2
import numpy as np

class ColorExtractor(Node):
    def __init__(self):
        super().__init__('color_extractor')
        self.publisher_ = self.create_publisher(ColorRGBA, 'led_color', 10)
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        # Get the dimensions of the frame
        height, width, _ = frame.shape

        # Define the region of interest (ROI) at the center of the frame
        start_x = width // 2 - 50
        start_y = height // 2 - 50
        end_x = start_x + 100
        end_y = start_y + 100

        # Extract the ROI
        roi = frame[start_y:end_y, start_x:end_x]

        # Calculate the average color in the ROI
        avg_color_per_row = np.average(roi, axis=0)
        avg_color = np.average(avg_color_per_row, axis=0)
        avg_color = avg_color.astype(int)

        # Create a ColorRGBA message
        color_msg = ColorRGBA()
        color_msg.r = avg_color[2]/255.0# Convert BGR to RGB and normalize
        color_msg.g = avg_color[1] /255.0
        color_msg.b = avg_color[0] /255.0
        color_msg.a = 1.0

        # Publish the color message
        self.publisher_.publish(color_msg)
        self.get_logger().info(f'Publishing color: R={color_msg.r}, G={color_msg.g}, B={color_msg.b}')

        # Display the ROI on the frame
        cv2.rectangle(frame, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    color_extractor = ColorExtractor()

    try:
        rclpy.spin(color_extractor)
    except KeyboardInterrupt:
        pass

    color_extractor.cap.release()
    cv2.destroyAllWindows()
    color_extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
