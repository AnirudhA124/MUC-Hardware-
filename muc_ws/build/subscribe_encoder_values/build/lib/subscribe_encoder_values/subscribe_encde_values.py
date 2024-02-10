#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class SubscribeEncoderValues(Node):
    def __init__(self):
        super().__init__('subscribe_encoder_values')
        self.subscription = self.create_subscription(
            Int32,
            'encoder_values_1',
            self.encoder_callback,
            10
        )
        
    def encoder_callback(self, msg):
        encoder_value = msg.data
        print("Received Encoder_1 Value:", encoder_value)
        # Add processing logic here
        # For example, you can store the value in a variable or perform calculations
        
def main(args=None):
    rclpy.init(args=args)
    subscribe_encoder_node = SubscribeEncoderValues()
    rclpy.spin(subscribe_encoder_node)
    subscribe_encoder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
