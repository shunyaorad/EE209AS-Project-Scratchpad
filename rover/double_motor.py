'''
Referenced from
http://www.knight-of-pi.org/simple-dc-motor-board-
for-the-raspberry-pi-with-ic-l293-and-software-
pulse-width-modulation/
'''

import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

class Motor:

    def __init__(self, pinForward, pinBackward, pinControl):
        """ Initialize the motor with its control pins and start pulse-width
             modulation """

        self.pinForward = pinForward
        self.pinBackward = pinBackward
        self.pinControl = pinControl
        GPIO.setup(self.pinForward, GPIO.OUT)
        GPIO.setup(self.pinBackward, GPIO.OUT)
        GPIO.setup(self.pinControl, GPIO.OUT)
        self.pwm_forward = GPIO.PWM(self.pinForward, 100)
        self.pwm_backward = GPIO.PWM(self.pinBackward, 100)
        self.pwm_forward.start(0)
        self.pwm_backward.start(0)
        GPIO.output(self.pinControl,GPIO.HIGH)

    def forward(self, speed):
        """ pinForward is the forward Pin, so we change its duty
             cycle according to speed. """
        self.pwm_backward.ChangeDutyCycle(0)
        self.pwm_forward.ChangeDutyCycle(speed)

    def backward(self, speed):
        """ pinBackward is the forward Pin, so we change its duty
             cycle according to speed. """

        self.pwm_forward.ChangeDutyCycle(0)
        self.pwm_backward.ChangeDutyCycle(speed)

    def stop(self):
        """ Set the duty cycle of both control pins to zero to stop the motor. """

        self.pwm_forward.ChangeDutyCycle(0)
        self.pwm_backward.ChangeDutyCycle(0)

# both motors share the same PWM pin
motor1 = Motor(7, 11, 12)
motor2 = Motor(16, 18, 12)

# Motor 1 test
motor1.forward(60)
sleep(2)
motor1.backward(60)
sleep(2)
motor1.stop()


# Motor 2 test
motor2.forward(60)
sleep(2)
motor2.backward(60)
sleep(2)
motor2.stop()

# Running both
motor1.forward(60)
motor2.forward(60)
sleep(2)
motor1.backward(60)
motor2.backward(60)
sleep(2)
motor1.stop()
motor2.stop()

GPIO.cleanup()
