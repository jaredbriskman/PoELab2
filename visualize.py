#!/usr/local/bin/python

from serial import Serial, SerialException

cxn = Serial('/dev/ttyACM0', baudrate=9600)
cur_value = ''
while(True):
    serial_val = cxn.read()
    if serial_val == b'\n':
        print(cur_value)
        cur_value = ''
    else:
        cur_value += str(cxn.read())
