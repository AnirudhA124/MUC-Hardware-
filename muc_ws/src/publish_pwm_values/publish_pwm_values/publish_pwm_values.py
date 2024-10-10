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
        self.publisher3 = self.create_publisher(Int32, 'pwm_values_3', 10)
        self.publisher4 = self.create_publisher(Int32, 'pwm_values_4', 10)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Define maximum PWM value
        max_pwm_value = 50  # Adjust this value based on your motor driver

        # Calculate the PWM values
        left_speed = linear_x + angular_z  # For motors 1 and 3
        right_speed = linear_x - angular_z  # For motors 2 and 4

        # Scale the PWM values to the appropriate range
        pwm_value1 = int(max(min(left_speed, 1.0) * max_pwm_value, -max_pwm_value))
        pwm_value2 = int(max(min(right_speed, 1.0) * max_pwm_value, -max_pwm_value))
        pwm_value3 = int(max(min(left_speed, 1.0) * max_pwm_value, -max_pwm_value))
        pwm_value4 = int(max(min(right_speed, 1.0) * max_pwm_value, -max_pwm_value))

        # Publish PWM values for each motor
        self.publish_pwm_values(pwm_value1, pwm_value2, pwm_value3, pwm_value4)

    def publish_pwm_values(self, pwm_value1, pwm_value2, pwm_value3, pwm_value4):
        self.publisher1.publish(Int32(data=pwm_value1))
        self.publisher2.publish(Int32(data=pwm_value2))
        self.publisher3.publish(Int32(data=pwm_value3))
        self.publisher4.publish(Int32(data=pwm_value4))

        self.get_logger().info('Publishing PWM values - Motor 1: %d, Motor 2: %d, Motor 3: %d, Motor 4: %d' % (pwm_value1, pwm_value2, pwm_value3, pwm_value4))

def main(args=None):
    rclpy.init(args=args)
    publish_pwm_node = PublishPWMValues()
    rclpy.spin(publish_pwm_node)
    publish_pwm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
