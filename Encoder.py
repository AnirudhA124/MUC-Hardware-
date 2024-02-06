import RPi.GPIO as GPIO
import time

Encoder1_output_A = 20
Encoder1_output_B = 21
Encoder2_output_A = 3
Encoder2_output_B = 4

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

def loop():
    while True:
        print("Result A: ", Count_pulses1)
        print("Result B: ", Count_pulses2)
        time.sleep(0.1)

def DC_Motor_Encoder1(channel):
    b = GPIO.input(Encoder1_output_B)
    global Count_pulses1
    if b > 0:
        Count_pulses1 += 1
    else:
        Count_pulses1 -= 1

def DC_Motor_Encoder2(channel):
    c = GPIO.input(Encoder2_output_B)
    global Count_pulses2
    if c > 0:
        Count_pulses2 += 1
    else:
        Count_pulses2 -= 1

if __name__ == "__main__":
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        GPIO.cleanup()
