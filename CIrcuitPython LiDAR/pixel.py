from adafruit_display_shapes.circle import Circle

class Pixel:
    def __init__(self, size, x, y):
        self.size = size    # Size of the pixel (radius)
        self.x = x          # Initial x-coordinate
        self.y = y          # Initial y-coordinate
        self.speed_x = 1    # Initial speed in the x-direction
        self.speed_y = 1    # Initial speed in the y-direction
        
        # Create a circle object for the ball
        self.circle = Circle(self.x, self.y, self.size, fill=0xFFFFFF)
        self.SCREEN_HEIGHT = 64 # Height of the scree
        self.SCREEN_WIDTH = 96  # Width of the screen
    
    def update(self, l, r, color):
        # Update the position and color of the pixel
        # The position is updated based on the provided l and r coordinates.
        # The color of the circle is updated to the provided color.
        
        # You can see some commented-out code for handling boundary and movement.
        # You can uncomment and modify it to control pixel movement and boundaries.
        
        # Check if the pixel has hit any boundaries and update its position accordingly.
        
#          if self.x <= left_paddle.x - self.size:
#             self.speed_x *= -1
# 
#          if self.x + self.size + right_paddle.width == right_paddle.x:
#             self.speed_x *= -1
#         
#          if self.x >= self.SCREEN_WIDTH-self.size*2 or self.x <= self.size:
#             self.speed_x *= -1
#             
#          if self.y >= self.SCREEN_HEIGHT-self.size*2 or self.y <= self.size:
#             self.speed_y *= -1
#             
#          self.x += self.speed_x
#          self.y += self.speed_y

        # Update the position of the circle       
         self.circle.x = l
         self.circle.y = r

        # Update the fill color of the circle
         self.circle.fill = color