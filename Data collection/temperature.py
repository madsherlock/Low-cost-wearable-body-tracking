# Saves sensor readings to "Temperature_<board id>_<time>.txt".
from visual import *
import serial
import string
import math
from struct import unpack
from time import time,sleep
from threading import Thread, Event

#User setup area
#   serial_port:    Serial port to which IMU is connected.
#   raw_data:       Output raw (not calibrated) values.
serial_port='COM29'
baud_rate=1000000
raw_data = True



# Check your COM port and baud rate
ser1 = serial.Serial(port=serial_port,baudrate=baud_rate, timeout=None, writeTimeout=None)
sleep(4)
ser1.flushInput()
ser1.write('#Mclit'); #Get board id
brd_id = ser1.readline()[:-2] #remove \n\r
print(brd_id)
f1 = open("Temperature_"+brd_id+"_"+str(time())+".txt", 'w')

def readthread(f,s):
    while(not endcondition.is_set()):
        f.write(s.readline())
    return

endcondition = Event()
def endOnInput(startstr,endstr):
    try:
        input(startstr)
    except SyntaxError:
        pass
    print endstr
    endcondition.set()
    return

def main():
    endcondition.clear()
    try:
        input("Press enter to start")
    except SyntaxError:
        pass
    
    print 'Started'
    ser1.write('#o0') #no output stream
    sleep(0.1)
    ser1.write('#oe0') #no error output
    sleep(0.1)
    ser1.write('#e1') #temperature output on
    sleep(0.1)
    ser1.write('#t0') #no counting missed loops
    sleep(0.1)
    ser1.write('#osrt' if raw_data else '#osct') #output sensor data RAW
    sleep(0.1)
    ft1 = Thread(target=readthread,args=(f1,ser1))
    ser1.write('#o1') #output stream on
    ft1.start()
    endOnInput("Press enter to stop","Stopped")
    ft1.join()
    ser1.write('#o0') #output stream off
    print 'Done'

main()
##try:
##    main()
##except:
##    print "error!"
ser1.close
f1.close()
