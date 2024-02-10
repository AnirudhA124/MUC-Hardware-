#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class PublishPWMValues(Node):
    def __init__(self):
        super().__init__('publish_pwm_values')
        self.publisher1 = self.create_publisher(Int32, 'pwm_values_1', 10)
        self.publisher2 = self.create_publisher(Int32, 'pwm_values_2', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate PWM values for both motors
        pwm_value1 = int(linear_x * 100 + angular_z * 100)  # Example calculation for motor 1
        pwm_value2 = int(linear_x * 100 - angular_z * 100)  # Example calculation for motor 2

        # Publish PWM values for each motor
        msg1 = Int32()
        msg1.data = pwm_value1
        self.publisher1.publish(msg1)

        msg2 = Int32()
        msg2.data = pwm_value2
        self.publisher2.publish(msg2)

        self.get_logger().info('Publishing PWM values - Motor 1: %d, Motor 2: %d' % (pwm_value1, pwm_value2))

def main(args=None):
    rclpy.init(args=args)
    publish_pwm_node = PublishPWMValues()
    rclpy.spin(publish_pwm_node)
    publish_pwm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

