from visual import *
import serial
import string
import math
from struct import unpack
from time import time,sleep
from threading import Thread, Event

measure_temperature = True
baud_rate=1000000
serial_port_1 = 'COM29'
serial_port_2 = 'COM32'
accel_offset_comp   = True
gyro_offset_comp    = True
gyro_temp_comp      = True
magn_hard_soft_comp = True
magn_temp_comp      = False

################################################################
tf = 4 if measure_temperature else 0
print('Opening serial ports. Please wait.')
ser1 = serial.Serial(port=serial_port_1,baudrate=baud_rate, timeout=None, writeTimeout=None)
ser2 = serial.Serial(port=serial_port_2,baudrate=baud_rate, timeout=None, writeTimeout=None)
sleep(4)
print('Serial ports open.')

#Writes same message to both serial ports
def serWrite(common_msg):
    ser1.write(common_msg)
    ser2.write(common_msg)

#Writes some information about data collection and sensor to a file
def fileHeader(ser,f):
    bigarray = []
    ser.write('#Mclb')
    bigarray.append(ser.read(1)) #brd_cal
    bigarray.append(ser.read(1)) #id0
    bigarray.append(ser.read(1)) #id1
    bigarray.append(ser.read(116)) #floats
    b_brd_cal = unpack('B',bigarray[0])[0]
    b_id0 = unpack('<1c',bigarray[1])[0]
    b_id1 = unpack('<1c',bigarray[2])[0]
    b_floats = unpack('<29f',bigarray[3])
    f.write('Accelerometer offset and scaling compensation: '+('Yes' if accel_offset_comp else 'No'))
    f.write('\nGyroscope offset and scaling compensation: '+('Yes' if gyro_offset_comp else 'No'))
    f.write('\nGyroscope temperature compensation: '+('Yes' if gyro_temp_comp else 'No'))
    f.write('\nMagnetometer hard and soft distortion compensation: '+('Yes' if magn_hard_soft_comp else 'No'))
    f.write('\nMagnetometer temperature compensation: '+('Yes' if magn_temp_comp else 'No'))
    f.write('\nBoard calibrated: '+('Yes' if b_brd_cal==1 else 'No'))
    f.write('\nBoard ID: '+b_id0+b_id1)
    f.write('\nAccelerometer X min / max: '+repr(b_floats[0])+' / '+repr(b_floats[1]))
    f.write('\nAccelerometer Y min / max: '+repr(b_floats[2])+' / '+repr(b_floats[3]))
    f.write('\nAccelerometer Z min / max: '+repr(b_floats[4])+' / '+repr(b_floats[5]))
    f.write('\nGyroscope offset X,Y,Z: '+repr(b_floats[6])+','+repr(b_floats[7])+','+repr(b_floats[8]))
    f.write('\nGyroscope working temperature: '+repr(b_floats[9]))
    f.write('\nGyroscope temperature sensitivity X,Y,Z: '+repr(b_floats[10])+','+
          repr(b_floats[11])+','+repr(b_floats[12]))
    f.write('\nMagnetometer ellipsoid center: {'+repr(b_floats[13])+','+
          repr(b_floats[14])+','+repr(b_floats[15])+'}')
    f.write('\nMagnetometer ellipsoid transform:\n{{'+repr(b_floats[16])+','+
          repr(b_floats[17])+','+repr(b_floats[18])+'},\n{'+
          repr(b_floats[19])+','+repr(b_floats[20])+','+
          repr(b_floats[21])+'},\n{'+repr(b_floats[22])+','+
          repr(b_floats[23])+','+repr(b_floats[24])+'}}')
    f.write('\nMagnetometer working temperature: '+repr(b_floats[25]))
    f.write('\nMagnetometer temperature sensitivity: {'+repr(b_floats[26])+','+
          repr(b_floats[27])+','+repr(b_floats[28])+'}\n')
    if(measure_temperature):
        f.write('temp,')
    f.write('ax,ay,az,mx,my,mz,gx,gy,gz\n')

serWrite('#o0') #Output stream OFF
serWrite('#oe0') #Error output OFF
ser1.flushInput() #Clear data from IMU
ser2.flushInput()
serWrite('#Mclit') #Get board id
brd_id_1 = ser1.readline()[:-2] #remove \n\r
brd_id_2 = ser2.readline()[:-2]
print('Board 1: '+brd_id_1+', board 2: '+brd_id_2)
the_time = str(time())
print('Opening files and writing headers.')
f1 = open("Serial_"+the_time+'_'+brd_id_1+".txt",'w')
f2 = open("Serial_"+the_time+'_'+brd_id_2+".txt",'w')
fileHeader(ser1,f1)
fileHeader(ser2,f2)
print('Files ready for data collection.')

bigarray1 = []
bigarray2 = []
#Unpacks collected data and writes to files
def arrayUnpack():
    global bigarray1, bigarray2
    for d in bigarray1:
        data1=unpack('<10f' if measure_temperature else '<9f',d[-(36+tf):])
        for i in data1[:-1]:
            f1.write(repr(i)+',')
        f1.write(repr(data1[-1])+'\n')
    for d in bigarray2:
        data1=unpack('<10f' if measure_temperature else '<9f',d[-(36+tf):])
        for i in data1[:-1]:
            f2.write(repr(i)+',')
        f2.write(repr(data1[-1])+'\n')

endcondition = Event()
#Thread that sets a flag when user presses enter.
def endOnInput(startstr,endstr):
    try:
        input(startstr)
    except SyntaxError:
        pass
    print endstr
    endcondition.set()
    return

#Data-collecting thread (one thread per sensor)
def reader(ser,bigarray):
    while(not endcondition.is_set()):
        bigarray.append(ser.read(36+tf))
    return

#Initializes data collection.
def main():
    endcondition.clear()
    t1 = Thread(target=endOnInput,args=("Press enter to stop","Stopped"))
    readt1=Thread(target=reader,args=(ser1,bigarray1))
    readt2=Thread(target=reader,args=(ser2,bigarray2))
    print("Make sure process is high priority")
    serWrite('#oscb') #Calibrated, binary data output
    serWrite('#t0') #No counting missed loops
    serWrite('#Mcoa1' if accel_offset_comp else '#Mcoa0') #Accelerometer offset and scaling compensation
    serWrite('#Mcog1' if gyro_offset_comp else '#Mcog0') #Gyro offset and scaling compensation
    serWrite('#Mcog3' if gyro_temp_comp else '#Mcog2') #Gyro temperature compensation
    serWrite('#Mcom1' if magn_hard_soft_comp else '#Mcom0') #Magnetometer hard & soft iron distortion compensation
    serWrite('#Mcom3' if magn_temp_comp else '#Mcom2') #Magnetometer temperature compensation
    serWrite('#e1' if measure_temperature else '#e0')
    try:
        input("Press enter to start")
    except SyntaxError:
        pass
    
    print 'Started'
    serWrite('#o1')
    t1.start()
    readt1.start()
    readt2.start()
    t1.join()
    readt1.join()
    readt2.join()
    print 'Unpacking data'
    arrayUnpack()
    serWrite('#o0')
    print 'Done'

main()
ser1.close
ser2.close
f1.close()
f2.close()
