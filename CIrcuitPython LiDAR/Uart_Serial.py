import busio
import board
import time
from binascii import hexlify


class UART_SERIAL:
 
    def __init__(self):
        # Initialize the UART serial communication with specified pins and baud rate
        self.serial = busio.UART(tx=board.GP8, rx=board.GP9, baudrate=115200)    #Define receiving interface of Lidar UART1
    
    def message(self,message):
        # Send a text message as ASCII data
        self.serial.write(bytes(f"<{message}>", "ascii"))
    
    def sendMessage(self, message):
        # Send a numeric message as a binary byte array
        hex_message = message.to_bytes(2,'big')
        self.serial.write(bytes(hex_message))
    
    def readLine(self):
        # Read and return a line of data from the serial interface
        return self.serial.readline()
    
    def readCommand(self, nbytes=None):
        # Read and return a specified number of bytes from the serial interface
        return self.serial.read(nbytes)
