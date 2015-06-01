from visual import *
import serial
import string
import math
from struct import unpack
from time import time,sleep
from threading import Thread, Event


# Check your COM port and baud rate
ser1 = serial.Serial(port='COM29',baudrate=1000000, timeout=None, writeTimeout=None)
ser2 = serial.Serial(port='COM31',baudrate=1000000, timeout=None, writeTimeout=None)
f1 = open("GyroBibl_"+str(time())+".txt", 'w')
f2 = open("GyroKjen_"+str(time())+".txt", 'w')

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
    #t1 = Thread(target=endOnInput,args=("Press enter to stop","Stopped"))
    try:
        input("Press enter to start")
    except SyntaxError:
        pass

    
    print 'Started'
    ser1.write('#o0#oe0#e2#t0')
    ser2.write('#o0#oe0#e2#t0')
    ft1 = Thread(target=readthread,args=(f1,ser1))
    ft2 = Thread(target=readthread,args=(f2,ser2))
    ser1.write('#o1')
    ser2.write('#o1')
    ft1.start()
    ft2.start()
    #t1.start()
    #while(not endcondition.is_set()):
    #    razorline2()
    endOnInput("Press enter to stop","Stopped")
    #while(not endcondition.is_set()):
    #    f1.write(ser1.readline())
    ft1.join()
    ft2.join()
    ser1.write('#o0')
    ser2.write('#o0')
    print 'Done'
try:
    main()
except:
    print "error!"
ser1.close
ser2.close
f1.close()
f2.close()
