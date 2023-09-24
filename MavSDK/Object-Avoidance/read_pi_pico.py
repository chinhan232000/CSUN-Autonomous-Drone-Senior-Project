#!/usr/bin/env python3

import serial
import asyncio
import serial_asyncio

class OutputProtocol(asyncio.Protocol):
    def __init__(self):
        self.distance = 0
        self.reader = None
        self.writer = None


    async def run(self, port='/dev/ttyACM0'):
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url=port, baudrate=115200, parity=serial.PARITY_NONE, 
                                                    stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        return self.reader, self.writer

    async def main(self):
        r, w = await self.run()
        while True:
            d = await r.readline()
            line = str(d[:-2], 'utf8')
            print(line)

if __name__=="__main__":
    connection = OutputProtocol()
    asyncio.run(connection.main())