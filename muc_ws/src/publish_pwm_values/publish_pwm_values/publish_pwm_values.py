#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class PublishPWMValues(Node):
    def __init__(self):
        super().__init__('publish_pwm_values')
        self.publisher1 = self.create_publisher(Int32, 'pwm_values_1', 10)
        self.publisher2 = self.create_publisher(Int32, 'pwm_values_2', 10)
        self.pwm_value1 = int(sys.argv[1]) if len(sys.argv) > 1 else 0
        self.pwm_value2 = int(sys.argv[2]) if len(sys.argv) > 2 else 0
        self.timer = self.create_timer(1.0, self.publish_pwm)

    def publish_pwm(self):
        msg1 = Int32()
        msg1.data = self.pwm_value1
        self.publisher1.publish(msg1)
        self.get_logger().info('Publishing PWM value 1: %d' % self.pwm_value1)

        msg2 = Int32()
        msg2.data = self.pwm_value2
        self.publisher2.publish(msg2)
        self.get_logger().info('Publishing PWM value 2: %d' % self.pwm_value2)

def main(args=None):
    rclpy.init(args=args)
    publish_pwm_node = PublishPWMValues()
    rclpy.spin(publish_pwm_node)
    publish_pwm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
