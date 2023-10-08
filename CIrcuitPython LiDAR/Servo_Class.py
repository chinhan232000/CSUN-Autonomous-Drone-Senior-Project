import time, random, board
from pwmio import PWMOut
from adafruit_motor import servo

class SERVO:
    
    def __init__(self):
        # Initialize the servo motor with PWMOut and set the min and max pulse values
        self.servoA = servo.Servo(PWMOut(board.GP15, frequency=50), min_pulse=500, max_pulse=2250)
    
    def moveToAngle(self, angle):
        # Move the servo to the specified angle
        self.servoA.angle = angle
    
    def moveSmoothToAngle(self, angle):
        # Smoothly move the servo to the specified angle using easing
        ani_pos = 0
        ease_speed = 0.1
        num_ease_slices = 10
        secs = 0.05

        # Perform the easing animation
        for i in range(num_ease_slices):
            # Calculate the new angle by applying easing
            self.servoA.angle += (angle - self.servoA.angle) * ease_speed

            # Pause for a fraction of a second to create a smooth animation
            time.sleep(secs/num_ease_slices)
        