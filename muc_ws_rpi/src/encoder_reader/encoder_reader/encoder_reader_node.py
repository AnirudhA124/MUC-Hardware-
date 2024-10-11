import RPi.GPIO as GPIO
import time
import rclpy
from std_msgs.msg import Int32

outcome = [0, -1, 1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]

# Define GPIO pins for encoders
Encoder1_output_A = 20
Encoder1_output_B = 16
Encoder2_output_A = 1
Encoder2_output_B = 10
Encoder3_output_A = 5
Encoder3_output_B = 6
Encoder4_output_A = 23
Encoder4_output_B = 24

Count_pulses1 = 0
Count_pulses2 = 0
Count_pulses3 = 0
Count_pulses4 = 0

last_AB1 = 0b00
last_AB2 = 0b00
last_AB3 = 0b00
last_AB4 = 0b00

def setup():
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Encoder1_output_A, GPIO.IN)
    GPIO.setup(Encoder1_output_B, GPIO.IN)
    GPIO.setup(Encoder2_output_A, GPIO.IN)
    GPIO.setup(Encoder2_output_B, GPIO.IN)
    GPIO.setup(Encoder3_output_A, GPIO.IN)
    GPIO.setup(Encoder3_output_B, GPIO.IN)
    GPIO.setup(Encoder4_output_A, GPIO.IN)
    GPIO.setup(Encoder4_output_B, GPIO.IN)

    # Adding event detection
    GPIO.add_event_detect(Encoder1_output_A, GPIO.BOTH, callback=DC_Motor_Encoder1)
    GPIO.add_event_detect(Encoder1_output_B, GPIO.BOTH, callback=DC_Motor_Encoder1)
    GPIO.add_event_detect(Encoder2_output_A, GPIO.BOTH, callback=DC_Motor_Encoder2)
    GPIO.add_event_detect(Encoder2_output_B, GPIO.BOTH, callback=DC_Motor_Encoder2)
    GPIO.add_event_detect(Encoder3_output_A, GPIO.BOTH, callback=DC_Motor_Encoder3)
    GPIO.add_event_detect(Encoder3_output_B, GPIO.BOTH, callback=DC_Motor_Encoder3)
    GPIO.add_event_detect(Encoder4_output_A, GPIO.BOTH, callback=DC_Motor_Encoder4)
    GPIO.add_event_detect(Encoder4_output_B, GPIO.BOTH, callback=DC_Motor_Encoder4)

def publish_encoder_values(node):
    global Count_pulses1, Count_pulses2, Count_pulses3, Count_pulses4
    publisher1 = node.create_publisher(Int32, 'encoder_values_1', 10)
    publisher2 = node.create_publisher(Int32, 'encoder_values_2', 10)
    publisher3 = node.create_publisher(Int32, 'encoder_values_3', 10)
    publisher4 = node.create_publisher(Int32, 'encoder_values_4', 10)
    logger = node.get_logger()

    while rclpy.ok():
        msg1 = Int32(data=Count_pulses1)
        publisher1.publish(msg1)
        logger.info(f'Encoder 1 value: {Count_pulses1}')

        msg2 = Int32(data=Count_pulses2)
        publisher2.publish(msg2)
        logger.info(f'Encoder 2 value: {Count_pulses2}')

        msg3 = Int32(data=Count_pulses3)
        publisher3.publish(msg3)
        logger.info(f'Encoder 3 value: {Count_pulses3}')

        msg4 = Int32(data=Count_pulses4)
        publisher4.publish(msg4)
        logger.info(f'Encoder 4 value: {Count_pulses4}')

        time.sleep(0.1)

def DC_Motor_Encoder1(channel):
    global Count_pulses1, last_AB1
    A = GPIO.input(Encoder1_output_A)
    B = GPIO.input(Encoder1_output_B)
    current_AB1 = (A << 1) | B
    position = (last_AB1 << 2) | current_AB1
    Count_pulses1 += outcome[position]
    last_AB1 = current_AB1

def DC_Motor_Encoder2(channel):
    global Count_pulses2, last_AB2
    A = GPIO.input(Encoder2_output_A)
    B = GPIO.input(Encoder2_output_B)
    current_AB2 = (A << 1) | B
    position = (last_AB2 << 2) | current_AB2
    Count_pulses2 += outcome[position]
    last_AB2 = current_AB2

def DC_Motor_Encoder3(channel):
    global Count_pulses3, last_AB3
    A = GPIO.input(Encoder3_output_A)
    B = GPIO.input(Encoder3_output_B)
    current_AB3 = (A << 1) | B
    position = (last_AB3 << 2) | current_AB3
    Count_pulses3 += outcome[position]
    last_AB3 = current_AB3

def DC_Motor_Encoder4(channel):
    global Count_pulses4, last_AB4
    A = GPIO.input(Encoder4_output_A)
    B = GPIO.input(Encoder4_output_B)
    current_AB4 = (A << 1) | B
    position = (last_AB4 << 2) | current_AB4
    Count_pulses4 += outcome[position]
    last_AB4 = current_AB4

def main():
    rclpy.init()
    node = rclpy.create_node('encoder_node')
    try:
        setup()
        publish_encoder_values(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
