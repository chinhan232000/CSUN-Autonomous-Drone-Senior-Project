# This script supports the Raspberry Pi Pico board and the Lilygo ESP32-S2 board
# Color OLED Display: http://educ8s.tv/part/ColorOLED
# Raspberry Pi Pico: http://educ8s.tv/part/RaspberryPiPico
# ESP32-S2 Board: http://educ8s.tv/part/esp32s2

import board, busio, displayio, os
import terminalio #Just a font
import time
from adafruit_ssd1331 import SSD1331
from adafruit_display_text import label
from pixel import Pixel

from Lidar_Class import LIDAR
from Servo_Class import SERVO

displayio.release_displays()

board_type = os.uname().machine
print(f"Board: {board_type}")

if 'Pico' in board_type:
    clk_pin, mosi_pin, reset_pin, dc_pin, cs_pin = board.GP18, board.GP19, board.GP16, board.GP20, board.GP17
elif 'ESP32-S2' in board_type:
    mosi_pin, clk_pin, reset_pin, cs_pin, dc_pin = board.IO35, board.IO36, board.IO38, board.IO34, board.IO37    
else:
    mosi_pin, clk_pin, reset_pin, cs_pin, dc_pin = board.GP11, board.GP10, board.GP17, board.GP18, board.GP16
    print("This board is not supported. Change the pin definitions above.")

spi = busio.SPI(clock=clk_pin, MOSI=mosi_pin)

display_bus = displayio.FourWire(spi, command=dc_pin, chip_select=cs_pin, reset=reset_pin)

display = SSD1331(display_bus, width=96, height=64)

lidar = LIDAR()
lidar.set_samp_rate()
lidar.save_settings()

servo1 = SERVO()
servo1.moveToAngle(0)
# Make the display context
splash = displayio.Group()
display.show(splash)

my_list =[]

for x in range(0, 90, 1): #range(start, end, step)
    pixel = Pixel(1, x+4, 45)
    my_list.append(pixel)
    splash.append(my_list[x].circle)
    

# my_list[10].update(12,30)
# my_list[11].update(13,5)
#pixel.update(50,50)
#pixel.update(10,30)
# color_bitmap = displayio.Bitmap(96, 64, 1)
# color_palette = displayio.Palette(1)
# color_palette[0] = 0x000000  # Bright Green
# 
# bg_sprite = displayio.TileGrid(color_bitmap, pixel_shader=color_palette, x=0, y=0)
# splash.append(bg_sprite)

# Draw a smaller inner rectangle
# inner_bitmap = displayio.Bitmap(86, 54, 1)
# inner_palette = displayio.Palette(1)
# inner_palette[0] = 0xAA0088  # Purple
# inner_sprite = displayio.TileGrid(inner_bitmap, pixel_shader=inner_palette, x=5, y=5)
# splash.append(inner_sprite)

# Draw a label

# distance = lidar.getLidarDistance()
# #print("D: {} cm S: {} T: {} *C\n".format(distance,strength,temperature))
# #text = "Ang:   Dist:  "
# text = "Ang:{} Dist:{}".format(0,distance)
# text_area = label.Label(terminalio.FONT, text=text, color=0xFFFFFF, x=1, y=55)
# splash.append(text_area)


distance = lidar.getLidarDistance()
#text = "Ang:{} Dist:{}".format(0,distance)
text_area = label.Label(terminalio.FONT, text="Ang:{} Dist:{}".format(0,distance), color=0xFFFFFF)

# set label position on the display
text_area.anchor_point = (0, 0)
text_area.anchored_position = (1, 50)


splash.append(text_area)
ang = 0
fw  = 1
while True:
    dist = lidar.getLidarDistance()
    #time.sleep(0.1)
    #lidar.getLidarDistanceInInch()
    #print("inch: {}".format((dist*2.54)/10))
    #print(lidar.getLidarDistanceInInch())
    #time.sleep(0.01)
    servo1.moveSmoothToAngle(ang)
    if dist > 0:
        text_area.text="Ang:{} Dist:{}".format(180 - ang,dist)
        #text_area.color = 0x00FFFF
        #print(dist%49)
        #print("Ang:{} Dist:{} form:{}".format(180 - ang,dist, (49 - (int)(dist/10))))
        #time.sleep(0.1)
        #my_list[10].update(12,dist)
        index = 90 - ((int)(ang/2)+1)
        #print(index)
        if dist < 100:
            my_list[index].update(index,(49 - (int)(dist/10)), 0xFC0303) #Red
        elif dist < 250:
            my_list[index].update(index,(49 - (int)(dist/10)), 0xFCF403) #Yellow
        else:
            my_list[index].update(index,(49 - (int)(dist/10)), 0x03FC03) #Green
            
    pass
    
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
    
    #time.sleep(1)