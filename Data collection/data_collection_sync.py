from visual import *
import serial
import string
import math
from struct import unpack
from time import time,sleep
from threading import Thread, Event

#grad2rad = 3.1415926535897932384626433832795/180.0


#If true: better data sync, risk of dropped samples
#If false: worse data sync, no dropped samples
# Only set to false if you know your serial communication is fast
#  - which you can check by setting to true and looking at first data column
count_missed_loops = False
measure_temperature = True
baud_rate=1000000
serial_port_1 = 'COM29'
serial_port_2 = 'COM31'

accel_offset_comp   = True
gyro_offset_comp    = True
gyro_temp_comp      = True
magn_hard_soft_comp = True
magn_temp_comp      = False





################################################################
tf = 4 if measure_temperature else 0

# Check your COM port and baud rate
print('Opening serial ports. Please wait.')
ser1 = serial.Serial(port=serial_port_1,baudrate=baud_rate, timeout=None, writeTimeout=None)
ser2 = serial.Serial(port=serial_port_2,baudrate=baud_rate, timeout=None, writeTimeout=None)
sleep(4)
print('Serial ports open.')
ser1.write('#o0')
ser2.write('#o0')
ser1.flushInput()
ser2.flushInput()

ser1.write('#Mclit') #Get board id
ser2.write('#Mclit')
brd_id_1 = ser1.readline()[:-2] #remove \n\r
brd_id_2 = ser2.readline()[:-2]

print('Board 1: '+brd_id_1+', board 2: '+brd_id_2)

f1 = open("Serial_"+brd_id_1+'_'+brd_id_2+'_'+str(time())+".txt", 'w')

bigarray1 = []
bigarray2 = []

def read_ser2():
    ser2.write("#m") # "#m" forces immediate sensor reading
    bigarray2.append(ser2.read(36+tf))
    return

def razorline1():
    #Read data almost synchronously
    t = Thread(target=read_ser2)
    t.start()
    if not count_missed_loops:
        bigarray1.append(ser1.read(36+tf))
    else:
        ser1.write("#f") # "#f" follows sensor sampling period
        bigarray1.append(ser1.read(37+tf))
    t.join()

def arrayUnpack():
    global bigarray1, bigarray2
    for d1,d2 in map(None,bigarray1,bigarray2):
        data1_1 = unpack('<10f' if measure_temperature else '<9f',d1[-(36+tf):])
        data2_1 = unpack('<10f' if measure_temperature else '<9f',d2[-(36+tf):])
        
        if count_missed_loops:
            missed_loops = unpack('B',d1[0])
            f1.write(repr(missed_loops[0])+',')
            
        for i in data1_1:
            f1.write(repr(i)+',')
        for i in data2_1[:-1]:
            f1.write(repr(i)+',')
        f1.write(repr(data2_1[-1])+'\n')

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
    t1 = Thread(target=endOnInput,args=("Press enter to stop","Stopped"))

    print("Make sure process is high priority")
    sleep(3)
    ser1.write('#oscb') #Calibrated, binary data output
    ser2.write('#oscb')
    ser1.write('#Mcoa1' if accel_offset_comp else '#Mcoa0') #Accelerometer offset and scaling compensation
    ser1.write('#Mcog1' if gyro_offset_comp else '#Mcog0') #Gyro offset and scaling compensation
    ser1.write('#Mcog3' if gyro_temp_comp else '#Mcog2') #Gyro temperature compensation
    ser1.write('#Mcom1' if magn_hard_soft_comp else '#Mcom0') #Magnetometer hard & soft iron distortion compensation
    ser1.write('#Mcom3' if magn_temp_comp else '#Mcom2') #Magnetometer temperature compensation
    ser2.write('#Mcoa1' if accel_offset_comp else '#Mcoa0') #Accelerometer offset and scaling compensation
    ser2.write('#Mcog1' if gyro_offset_comp else '#Mcog0') #Gyro offset and scaling compensation
    ser2.write('#Mcog3' if gyro_temp_comp else '#Mcog2') #Gyro temperature compensation
    ser2.write('#Mcom1' if magn_hard_soft_comp else '#Mcom0') #Magnetometer hard & soft iron distortion compensation
    ser2.write('#Mcom3' if magn_temp_comp else '#Mcom2') #Magnetometer temperature compensation
    if measure_temperature:
        ser1.write('#e1')
        ser2.write('#e1')
    else:
        ser1.write('#e0')
        ser2.write('#e0')
    
    try:
        input("Press enter to start")
    except SyntaxError:
        pass

    
    print 'Started'
    ser1.write('#t1' if count_missed_loops else '#t0#o1')
    t1.start()
    while(not endcondition.is_set()):
        razorline1()
    print 'Unpacking data'
    arrayUnpack()
    print 'Done'

main()
ser1.close
ser2.close
f1.close()
