import busio
import board
import time
from binascii import hexlify


class SERIAL:
 
    def __init__(self):
        self.serial = busio.UART(tx=board.GP8, rx=board.GP9, baudrate=115200)    #Define receiving interface of Lidar UART1
    
    def message(self,message):
        self.serial.write(bytes(f"<{message}>", "ascii"))
    
    def sendMessage(self, message):
        hex_message = message.to_bytes(2,'big')
        self.serial.write(bytes(hex_message))
    
    def readLine(self):
        return self.serial.readline()
    
    def readCommand(self, nbytes=None):
        return self.serial.read(nbytes)