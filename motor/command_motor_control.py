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

try:
    while True:
        cmd = raw_input("Command, a: right, s:left, w:both, x:stop:, speed:0, 1, 2: ")
        if len(cmd) > 0:
            direction = cmd[0]
            speed = cmd[1:]
            if speed == "0":
                vel = 50
            elif speed == "1":
                vel = 60
            else:
                vel = 70
        if direction == "a":
            print "right motor"
            motor1.forward(vel)
            motor2.stop()
        if direction == "s":
            print "left motor"
            motor2.forward(vel)
            motor1.stop()
        if direction == "w":
            print "both motor"
            motor1.forward(vel)
            motor2.forward(vel)
        if direction == "x":
            print "stopped"
            motor1.stop()
            motor2.stop()

except KeyboardInterrupt:
    motor1.stop()
    motor2.stop()
    print "\nkeyboard interrupt, stopped"
    GPIO.cleanup()