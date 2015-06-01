from visual import *
import serial
import string
import math
from struct import unpack,pack
from time import time,sleep
from threading import Thread, Event

def packAsFloat(value):
    return pack('f',value)

# Check your COM port and baud rate
ser1 = serial.Serial(port='COM29',baudrate=1000000, timeout=None, writeTimeout=None)
sleep(3)

tf=1.12
ser1.write('#o0#oe0#Mtf')

ser1.write(packAsFloat(tf))
print ser1.readline()

ser1.close
