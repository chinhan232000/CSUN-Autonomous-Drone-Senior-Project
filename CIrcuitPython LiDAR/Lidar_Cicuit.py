#from machine import UART, Pin
import busio
import board
import time
from binascii import hexlify

#lidar = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))    #Define receiving interface of Lidar
#lidar = UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13))    #Define receiving interface of Lidar UART0

# Initialize UART communication
lidar = busio.UART(tx=board.GP12, rx=board.GP13, baudrate=115200)    #Define receiving interface of Lidar UART0

print(lidar)

time.sleep(0.001)

# Define Lidar data packet structure
"""
byte[0] = 0x59
byte[1] = 0x59
byte[2] = Dist_L
byte[3] = Dist_H
byte[4] = Strength_L
byte[5] = Strength_H
byte[6] = Temp_L
byte[7] = Temp_H
byte[8] = Checksum
total = byte[0]+byte[1]+byte[2]+byte[3]+byte[4]+byte[5]+byte[6]+byte[7]
the lower 8 bits [LSB] of the total is the checksum 

"""

# Define the default frame rate
frame_rate = 15 # default at 20Hz (best tested at 115200)

def save_settings():
    # Save Lidar settings
    print("\nSaving setting...")
    info_packet = [0x5a,0x04,0x11,0x6F]
    lidar.write(bytes(info_packet))
    time.sleep(.0001)
    
def set_samp_rate(samp_rate=frame_rate):
    # change the sample rate
    global frame_rate
    print("Setting sample rate to {} Hz".format(samp_rate))
    samp_rate = int(samp_rate)
    frame_rate = samp_rate
    hex_rate = samp_rate.to_bytes(2,'big')
    print("hex_rate:{}".format(hex_rate)) # \xHH\xLL => hex_rate[0]=\xHH hex_rate[1]=\xLL 
    samp_rate_packet = [0x5a,0x06,0x03,hex_rate[1],hex_rate[0],00,00] # sample rate byte array
    for i in range(len(samp_rate_packet)):
        print(hex(samp_rate_packet[i]), end=" ")
    lidar.write(bytes(samp_rate_packet)) # send sample rate instruction
    time.sleep(0.001) # wait for change to take effect
    save_settings()
    return
# 
def get_version(UART0):
    # get version info | source: https://makersportal.com/blog/distance-detection-with-the-tf-luna-lidar-and-raspberry-pi
    info_packet = [0x5a,0x04,0x14,0x00]
    lidar.write(bytes(info_packet))
    start_tick = time.time()
    while (time.time()-start_tick < 10):
#         print('.',end = '')
#         utime.sleep_ms(1)
        bin_ascii = bytearray()
        #if UART0.any() > 0:
        if UART0.in_waiting > 0:
            bin_ascii += UART0.read(30) # known 30 bytes-length response
            if bin_ascii[0] == 0x5a:
                hex_stream = hexlify(bin_ascii,' ').decode('utf-8')
                hex_array = hex_stream.split(" ")
                print(hex_stream) # uncomment this to see the HEX data
                print("{}".format(bin_ascii.decode('utf-8')))
                version = bin_ascii[0:].decode('utf-8')
                lst = []
                for c in version:
                    lst.append(c)
                
                for i in range(len(hex_array)):
                    print("0x{} : ascii -> {}".format(hex_array[i],lst[i]))
                print('\nVersion -'+version+'\n')
                return
            else:
                lidar.write(bytes(info_packet))
    print("Failed to retrieve version. This is a bit of a hit or miss kind of thing...")
#
def getLidarDistance(UART0):
    # Get Lidar distance data
    bin_ascii = bytearray()
    if UART0.in_waiting >0:
        bin_ascii += UART0.read(9)
        
        if bin_ascii[0] == 0x59 and bin_ascii[1] == 0x59:
            distance = bin_ascii[2] + bin_ascii[3] *256
        UART0.reset_input_buffer()
    return distance

def getLidarData(UART0):
    # Get Lidar data including distance, strength, and temperature   
    bin_ascii = bytearray()

    if UART0.in_waiting > 0:
        bin_ascii += UART0.read(9) # byte [0-8] = 9 bytes
        #hex_stream = hexlify(bin_ascii,' ').decode()
        #print(hex_stream[0:26]) # uncomment this to see the HEX data

        if bin_ascii[0] == 0x59 and bin_ascii[1] == 0x59 :
            distance   = bin_ascii[2] + bin_ascii[3] * 256             #Get distance value  
            strength    = bin_ascii[4] + bin_ascii[5] * 256            #Get Strength value  
            temperature= (bin_ascii[6] + bin_ascii[7]* 256)/8-256      #Get IC temperature value
            """
            bear in mind that serial printing at 20-30Hz drastically slowing down the entire process
            please use other means of displaying the data using components such as:
            i2c 16x2 LCD, i2c SSD1306 OLED, 7-segment TM1637, Dot Matrix MAX7219 
            """
            print("D: {} cm S: {} T: {} *C\n".format(distance,strength,temperature))
        UART0.reset_input_buffer()
        #time.sleep(0.01)

# try:
#     print("getting version")
#     get_version(lidar)
#     time.sleep(1)
#     # print("set sample rate via user input")
#     # rate = input("Please enter 1-30 Hz for Raspi Pico:")
#     # set_samp_rate(rate)
#     """
#     TFmini-Plus readout on Raspberry Pi Pico does not work responsively
#     with frame rate set to anything higher than 30Hz at 115200 baud rate.
#     
#     Generally:
#     1Hz-5Hz		usable for non-critical range sensing (slow response application)
#     6Hz-9Hz		good frame rate for semi-critical range sensing
#     10Hz-30Hz 	best for super critical range sensing
# 
#     """
#     set_samp_rate(10) # min: 1 - max: 30 
#     print("\nStart measuring")
#     while True:
#         getLidarData(lidar)
#         
# except KeyboardInterrupt:
#     print("Program Stopped")