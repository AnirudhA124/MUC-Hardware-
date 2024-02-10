#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class SubscribeEncoderValues(Node):
    def __init__(self):
        super().__init__('subscribe_encoder_values')
        self.subscription_1 = self.create_subscription(
            Int32,
            'encoder_values_1',
            self.encoder_callback_1,
            10
        )
        self.subscription_2 = self.create_subscription(
            Int32,
            'encoder_values_2',
            self.encoder_callback_2,
            10
        )
        
    def encoder_callback_1(self, msg):
        encoder_value_1 = msg.data
        self.get_logger().info("Received Encoder_1 Value: %d" % encoder_value_1)
        # Add processing logic here for encoder 1
        
    def encoder_callback_2(self, msg):
        encoder_value_2 = msg.data
        self.get_logger().info("Received Encoder_2 Value: %d" % encoder_value_2)
        # Add processing logic here for encoder 2
        
def main(args=None):
    rclpy.init(args=args)
    subscribe_encoder_node = SubscribeEncoderValues()
    rclpy.spin(subscribe_encoder_node)
    subscribe_encoder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
