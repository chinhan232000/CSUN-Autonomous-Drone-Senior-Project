# receiver.py / USB => USB
import os
from rusb import USB
from _thread import start_new_thread
from time import sleep_ms

usb = USB()

input_msg = None
bufferSTDINthread = start_new_thread(usb.bufferSTDIN, ())

while True:
  input_msg = usb.getLineBuffer()
  print(input_msg)
  sleep_ms(10)
