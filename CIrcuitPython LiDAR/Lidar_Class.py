import busio
import board
import time
from binascii import hexlify


class LIDAR:
    
    frame_rate = 20 # default at 20Hz (best tested at 115200)
    #lidar = busio.UART(tx=board.GP12, rx=board.GP13, baudrate=115200)
    
    def __init__(self):
        self.lidar = busio.UART(tx=board.GP12, rx=board.GP13, baudrate=115200)    #Define receiving interface of Lidar UART0
        
    def save_settings(self):
        # Save Lidar settings
        print("\nSaving setting...")
        info_packet = [0x5a,0x04,0x11,0x6F]
        self.lidar.write(bytes(info_packet))
        time.sleep(.0001)
    
    def set_samp_rate(self, samp_rate=frame_rate):
        # change the sample rate
        global frame_rate
        #global lidar
        print("Setting sample rate to {} Hz".format(samp_rate))
        samp_rate = int(samp_rate)
        frame_rate = samp_rate
        hex_rate = samp_rate.to_bytes(2,'big')
        print("hex_rate:{}".format(hex_rate)) # \xHH\xLL => hex_rate[0]=\xHH hex_rate[1]=\xLL 
        samp_rate_packet = [0x5a,0x06,0x03,hex_rate[1],hex_rate[0],00,00] # sample rate byte array
        for i in range(len(samp_rate_packet)):
            print(hex(samp_rate_packet[i]), end=" ")
        self.lidar.write(bytes(samp_rate_packet)) # send sample rate instruction
        time.sleep(0.001) # wait for change to take effect
        #save_settings()
        return
    
    def getLidarDistance(self):
        # Get Lidar distance data
        bin_ascii = bytearray()
        distance=0
        if self.lidar.in_waiting > 0:
            bin_ascii += self.lidar.read(9)        
            if bin_ascii[0] == 0x59 and bin_ascii[1] == 0x59:
                distance = bin_ascii[2] + bin_ascii[3] *256
        self.lidar.reset_input_buffer()
        #print("Value: %.2f" % distance)
        return distance
    
    
    def getLidarDistanceInInch(self):
        # Get Lidar distance in inches
        return (self.getLidarDistance() * 2.54)/10