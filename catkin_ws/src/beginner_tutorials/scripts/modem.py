#!/usr/bin/env python

import serial
ser = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=1,  rtscts=False, dsrdtr=False)
cmd = "AT+CREG?\r"
ser.write(cmd.encode())
msg = ser.read(64)
print(msg)