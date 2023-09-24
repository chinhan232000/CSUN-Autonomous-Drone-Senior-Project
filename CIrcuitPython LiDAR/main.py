import board, busio, displayio, os
import terminalio #Just a font
import time

from adafruit_ssd1331 import SSD1331
from adafruit_display_text import label
from pixel import Pixel
from Lidar_Class import LIDAR
from Servo_Class import SERVO
from Uart_Serial import UART_SERIAL

displayio.release_displays()

board_type = os.uname().machine
print(f"Board: {board_type}")

clk_pin, mosi_pin, reset_pin, dc_pin, cs_pin = board.GP18, board.GP19, board.GP16, board.GP20, board.GP17
spi = busio.SPI(clock=clk_pin, MOSI=mosi_pin)
display_bus = displayio.FourWire(spi, command=dc_pin, chip_select=cs_pin, reset=reset_pin)
display = SSD1331(display_bus, width=96, height=64)

# Configure LiDAR
lidar = LIDAR()
lidar.set_samp_rate()
lidar.save_settings()

servo1 = SERVO()
servo1.moveToAngle(0)

#serial = UART_SERIAL()

# Make the display context
splash = displayio.Group()
display.show(splash)

my_list =[]

#pixel Array
for x in range(0, 90, 1): #range(start, end, step)
    pixel = Pixel(1, x+4, 45)
    my_list.append(pixel)
    splash.append(my_list[x].circle)

text_area = label.Label(terminalio.FONT, text="Ang:{} Dist:{}".format(0,0), color=0xFFFFFF)

# set label position on the display
text_area.anchor_point = (0, 0)
text_area.anchored_position = (1, 50)


splash.append(text_area)
ang = 0
fw  = 1
while True:
    dist = lidar.getLidarDistance()
    
    if dist > 0:
        text_area.text="Ang:{} Dist:{}".format(180 - ang,dist)
        index = 90 - ((int)(ang/2)+1)
        
        print(dist)
        #serial.message(dist)
        
        if dist < 100: #limit in cm
            my_list[index].update(index,(49 - (int)(dist/10)), 0xFC0303) #Red
        elif dist < 250:
            my_list[index].update(index,(49 - (int)(dist/10)), 0xFCF403) #Yellow
        else:
            my_list[index].update(index,(49 - (int)(dist/10)), 0x03FC03) #Green     
    #pass    
        if fw == 1:
            ang = ang + 2
        else:
            ang = ang - 2
        if ang >= 180:
            fw = 0
            ang = 178
        if ang <= 0 :
            fw = 1
            ang = 2
        servo1.moveSmoothToAngle(ang)
