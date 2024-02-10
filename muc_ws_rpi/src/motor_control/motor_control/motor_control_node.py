#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import lgpio

class MotorController:
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm_pin = pwm_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin

        self.h = lgpio.gpiochip_open(0)  # Open GPIO chip

        # Claim GPIO pins for output
        lgpio.gpio_claim_output(self.h, pwm_pin)
        lgpio.gpio_claim_output(self.h, in1_pin)
        lgpio.gpio_claim_output(self.h, in2_pin)

        self.pwm = lgpio.tx_pwm(self.h, pwm_pin, 10000, 0)  # Start PWM with duty cycle 0, frequency 100000 (100Hz)

    def set_speed(self, speed):
        lgpio.tx_pwm(self.h, self.pwm_pin, 10000, speed)

    def forward(self):
        lgpio.gpio_write(self.h, self.in1_pin, 0)
        lgpio.gpio_write(self.h, self.in2_pin, 1)

    def backward(self):
        lgpio.gpio_write(self.h, self.in1_pin, 1)
        lgpio.gpio_write(self.h, self.in2_pin, 0)

    def stop(self):
        lgpio.gpio_write(self.h, self.in1_pin, 0)
        lgpio.gpio_write(self.h, self.in2_pin, 0)
        lgpio.tx_pwm(self.h, self.pwm_pin, 10000, 0)

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.motor1 = MotorController(pwm_pin=13, in1_pin=5, in2_pin=6)
        self.motor2 = MotorController(pwm_pin=18, in1_pin=17, in2_pin=27)
        
        self.subscription1 = self.create_subscription(
            Int32,
            'pwm_values_1',
            self.pwm_callback1,
            10
        )
        
        self.subscription2 = self.create_subscription(
            Int32,
            'pwm_values_2',
            self.pwm_callback2,
            10
        )

    def pwm_callback1(self, msg):
        pwm_value = msg.data
        self.get_logger().info("Received PWM Value for Motor 1: %d" % pwm_value)
        if pwm_value > 0:
            self.motor1.forward()
        elif pwm_value < 0:
            self.motor1.backward()
        else:
            self.motor1.stop()
        self.motor1.set_speed(abs(pwm_value))
        
    def pwm_callback2(self, msg):
        pwm_value = msg.data
        self.get_logger().info("Received PWM Value for Motor 2: %d" % pwm_value)
        if pwm_value > 0:
            self.motor2.forward()
        elif pwm_value < 0:
            self.motor2.backward()
        else:
            self.motor2.stop()
        self.motor2.set_speed(abs(pwm_value))

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
