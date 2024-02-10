import RPi.GPIO as GPIO
import time
import rclpy
from std_msgs.msg import Int32

Encoder1_output_A = 20
Encoder1_output_B = 21
Encoder2_output_A = 4
Encoder2_output_B = 3

Count_pulses1 = 0
Count_pulses2 = 0

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Encoder1_output_A, GPIO.IN)
    GPIO.setup(Encoder1_output_B, GPIO.IN)
    GPIO.setup(Encoder2_output_A, GPIO.IN)
    GPIO.setup(Encoder2_output_B, GPIO.IN)
    
    GPIO.add_event_detect(Encoder1_output_A, GPIO.RISING, callback=DC_Motor_Encoder1)
    GPIO.add_event_detect(Encoder2_output_A, GPIO.RISING, callback=DC_Motor_Encoder2)

def publish_encoder_values(node):
    global Count_pulses1, Count_pulses2
    publisher1 = node.create_publisher(Int32, 'encoder_values_1', 10)
    publisher2 = node.create_publisher(Int32, 'encoder_values_2', 10)
    logger = node.get_logger()

    while True:
        msg1 = Int32()
        msg1.data = Count_pulses1
        publisher1.publish(msg1)
        logger.info('Encoder 1 value: %d' % Count_pulses1)

        msg2 = Int32()
        msg2.data = Count_pulses2
        publisher2.publish(msg2)
        logger.info('Encoder 2 value: %d' % Count_pulses2)

        time.sleep(0.1)

def DC_Motor_Encoder1(channel):
    global Count_pulses1
    b = GPIO.input(Encoder1_output_B)
    if b > 0:
        Count_pulses1 += 1
    else:
        Count_pulses1 -= 1

def DC_Motor_Encoder2(channel):
    global Count_pulses2
    c = GPIO.input(Encoder2_output_B)
    if c > 0:
        Count_pulses2 += 1
    else:
        Count_pulses2 -= 1

def main():
    rclpy.init()
    node = rclpy.create_node('encoder_node')
    try:
        setup()
        publish_encoder_values(node)
    except KeyboardInterrupt:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
