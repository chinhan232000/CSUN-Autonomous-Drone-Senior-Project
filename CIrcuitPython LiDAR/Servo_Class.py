import time, random, board
from pwmio import PWMOut
from adafruit_motor import servo

class SERVO:
    
    def __init__(self):
        self.servoA = servo.Servo(PWMOut(board.GP15, frequency=50), min_pulse=500, max_pulse=2250)
    
    def moveToAngle(self, angle):
        self.servoA.angle = angle
    
    def moveSmoothToAngle(self, angle):
        ani_pos = 0
        ease_speed = 0.1
        num_ease_slices = 10
        secs = 0.05
        
        for i in range(num_ease_slices):
            self.servoA.angle += (angle - self.servoA.angle) * ease_speed
            time.sleep(secs/num_ease_slices)
        