#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import lgpio

class MotorController:
    def __init__(self, pwm_pin, in_pin):
        self.pwm_pin = pwm_pin
        self.in_pin = in_pin

        self.h = lgpio.gpiochip_open(0)  # Open GPIO chip

        # Claim GPIO pins for output
        lgpio.gpio_claim_output(self.h, pwm_pin)
        lgpio.gpio_claim_output(self.h, in_pin)

        self.pwm = lgpio.tx_pwm(self.h, pwm_pin, 10000, 0)  # Start PWM with duty cycle 0, frequency 100000 (100Hz)

    def set_speed(self, speed):
        lgpio.tx_pwm(self.h, self.pwm_pin, 10000, speed)

    def forward(self):
        lgpio.gpio_write(self.h, self.in_pin, 1)

    def backward(self):
        lgpio.gpio_write(self.h, self.in_pin, 0)

    def stop(self):
        lgpio.gpio_write(self.h, self.in_pin, 0)
        lgpio.tx_pwm(self.h, self.pwm_pin, 10000, 0)

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.motor1 = MotorController(pwm_pin=13, in_pin=5)
        self.motor2 = MotorController(pwm_pin=26, in_pin=19)
        self.motor3 = MotorController(pwm_pin=1, in_pin=16)
        self.motor4 = MotorController(pwm_pin=11, in_pin=10)
        
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
        
        self.subscription3 = self.create_subscription(
            Int32,
            'pwm_values_3',
            self.pwm_callback3,
            10
        )
        
        self.subscription4 = self.create_subscription(
            Int32,
            'pwm_values_4',
            self.pwm_callback4,
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
        
    def pwm_callback3(self, msg):
        pwm_value = msg.data
        self.get_logger().info("Received PWM Value for Motor 3: %d" % pwm_value)
        if pwm_value > 0:
            self.motor3.forward()
        elif pwm_value < 0:
            self.motor3.backward()
        else:
            self.motor3.stop()
        self.motor3.set_speed(abs(pwm_value))
    
    def pwm_callback4(self, msg):
        pwm_value = msg.data
        self.get_logger().info("Received PWM Value for Motor 4: %d" % pwm_value)
        if pwm_value > 0:
            self.motor4.forward()
        elif pwm_value < 0:
            self.motor4.backward()
        else:
            self.motor4.stop()
        self.motor4.set_speed(abs(pwm_value))

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
