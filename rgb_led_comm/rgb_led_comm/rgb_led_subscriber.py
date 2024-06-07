#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
import serial

class RGBLedSubscriber(Node):
    def __init__(self):
        super().__init__('rgb_led_subscriber')
        self.subscription = self.create_subscription(
            ColorRGBA,
            'led_color',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)

    def listener_callback(self, msg):
        red = int(msg.r * 255)
        green = int(msg.g * 255)
        blue = int(msg.b * 255)
        self.get_logger().info(f'Received color - R: {red}, G: {green}, B: {blue}')

        # Send the RGB values to the Arduino
        self.serial_port.write(bytes([red, green, blue]))

def main(args=None):
    rclpy.init(args=args)
    rgb_led_subscriber = RGBLedSubscriber()

    try:
        rclpy.spin(rgb_led_subscriber)
    except KeyboardInterrupt:
        pass

    rgb_led_subscriber.serial_port.close()
    rgb_led_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
