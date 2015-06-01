import serial
from struct import unpack,pack
from time import sleep

serial_port='COM29'


# Calibration parameters
# Bibliotek board
brd_cal = 1
id0 = 'b'
id1 = 'i'
accel_min_x = -270.0
accel_max_x =  260.0
accel_min_y = -253.0
accel_max_y =  264.0
accel_min_z = -264.0
accel_max_z =  244.0
#bi
gyro_offset_x = 8.5534897260611
gyro_offset_y = -1.0483657731485
gyro_offset_z = -6.6021161773221
gyro_working_temperature = 27.0
gyro_temperature_sensitivity_x = 1.6101594084482
gyro_temperature_sensitivity_y = 0.34858360861777
gyro_temperature_sensitivity_z = -0.47748384606518
magn_temperature_sensitivity_x = -0.56177638265379
magn_temperature_sensitivity_y = 0.48387583017262
magn_temperature_sensitivity_z = 1.6289980329712
#gyro_time_variance_x = 0.68521718710764
#gyro_time_variance_y = 0.75400783227888
#gyro_time_variance_z = 0.52886686970828
#magn_time_variance_x = 2.6919506620929
#magn_time_variance_y = 2.8808530867858
#magn_time_variance_z = 3.5746200538334
#acc_time_variance_x = 2.0407347445276
#acc_time_variance_y = 2.3999029546286
#acc_time_variance_z = 4.5954596365228


# Magnetic field at school
magn_ellipsoid_center_x = 147.989060180
magn_ellipsoid_center_y = 9.69557446140
magn_ellipsoid_center_z = -10.7002601010
magn_ellipsoid_transform_0_0 = 0.948804665145
magn_ellipsoid_transform_0_1 = 0.00847311251571
magn_ellipsoid_transform_0_2 = 0.00138356031675
magn_ellipsoid_transform_1_0 = 0.00847311251571
magn_ellipsoid_transform_1_1 = 0.946328120372
magn_ellipsoid_transform_1_2 = 0.0144306661663
magn_ellipsoid_transform_2_0 = 0.00138356031675
magn_ellipsoid_transform_2_1 = 0.0144306661663
magn_ellipsoid_transform_2_2 = 0.995851123530
magn_working_temperature = 28.3377647400

def packAsUchar(value):
    return pack('B',value)
def packAsChar(value):
    return pack('c',value)
def packAsFloat(value):
    return pack('f',value)

# Check your COM port and baud rate
print 'Please wait while serial port is being opened.'
ser1 = serial.Serial(port=serial_port,baudrate=1000000, timeout=None, writeTimeout=None)
sleep(3)
print 'Serial port opened. Writing quiet sequence.'
ser1.write('#o0#oe0')
print 'Flushing input buffer.'
ser1.flushInput()
print 'Input buffer flushed.'
print 'Sending calibration parameters.'


ser1.write('#Mcsb')
ser1.write(packAsUchar(brd_cal))
ser1.write(packAsChar(id0))
ser1.write(packAsChar(id1))
ser1.write(packAsFloat(accel_min_x))
ser1.write(packAsFloat(accel_min_y))
ser1.write(packAsFloat(accel_min_z))
ser1.write(packAsFloat(accel_max_x))
ser1.write(packAsFloat(accel_max_y))
ser1.write(packAsFloat(accel_max_z))
ser1.write(packAsFloat(gyro_offset_x))
ser1.write(packAsFloat(gyro_offset_y))
ser1.write(packAsFloat(gyro_offset_z))
ser1.write(packAsFloat(gyro_working_temperature))
ser1.write(packAsFloat(gyro_temperature_sensitivity_x))
ser1.write(packAsFloat(gyro_temperature_sensitivity_y))
ser1.write(packAsFloat(gyro_temperature_sensitivity_z))
ser1.write(packAsFloat(magn_ellipsoid_center_x))
ser1.write(packAsFloat(magn_ellipsoid_center_y))
ser1.write(packAsFloat(magn_ellipsoid_center_z))
ser1.write(packAsFloat(magn_ellipsoid_transform_0_0))
ser1.write(packAsFloat(magn_ellipsoid_transform_0_1))
ser1.write(packAsFloat(magn_ellipsoid_transform_0_2))
ser1.write(packAsFloat(magn_ellipsoid_transform_1_0))
ser1.write(packAsFloat(magn_ellipsoid_transform_1_1))
ser1.write(packAsFloat(magn_ellipsoid_transform_1_2))
ser1.write(packAsFloat(magn_ellipsoid_transform_2_0))
ser1.write(packAsFloat(magn_ellipsoid_transform_2_1))
ser1.write(packAsFloat(magn_ellipsoid_transform_2_2))
ser1.write(packAsFloat(magn_working_temperature))
ser1.write(packAsFloat(magn_temperature_sensitivity_x))
ser1.write(packAsFloat(magn_temperature_sensitivity_y))
ser1.write(packAsFloat(magn_temperature_sensitivity_z))


print 'Reading back calibration parameters (text):'
ser1.write('#Mclt')
print ser1.readline()[:-1] #Board calibrated
print ser1.readline()[:-1] #Board ID
print ser1.readline()[:-1] #Accelerometer X min / max
print ser1.readline()[:-1] #Accelerometer Y min / max
print ser1.readline()[:-1] #Accelerometer Z min / max
print ser1.readline()[:-1] #Gyroscope offset X,Y,Z
print ser1.readline()[:-1] #Gyroscope working temperature
print ser1.readline()[:-1] #Gyroscope temperature sensitivity X,Y,Z
print ser1.readline()[:-1] #Magnetometer ellipsoid center
print ser1.readline()[:-1] #Magnetometer ellipsoid transform
print ser1.readline()[:-1] #Magnetometer working temperature
print ser1.readline()[:-1] #Magnetometer temperature sensitivity


print 'Reading back calibration parameters (binary).'
ser1.write('#Mclb')
bigarray1 = []
bigarray1.append(ser1.read(1)) #brd_cal
bigarray1.append(ser1.read(1)) #id0
bigarray1.append(ser1.read(1)) #id1
bigarray1.append(ser1.read(116)) #floats
print 'Parameters have been read:'
b_brd_cal = unpack('B',bigarray1[0])[0]
b_id0 = unpack('<1c',bigarray1[1])[0]
b_id1 = unpack('<1c',bigarray1[2])[0]
b_floats = unpack('<29f',bigarray1[3])
b_accel_min_x = b_floats[0]
b_accel_max_x = b_floats[1]
b_accel_min_y = b_floats[2]
b_accel_max_y = b_floats[3]
b_accel_min_z = b_floats[4]
b_accel_max_z = b_floats[5]
b_gyro_offset_x = b_floats[6]
b_gyro_offset_y = b_floats[7]
b_gyro_offset_z = b_floats[8]
b_gyro_working_temperature = b_floats[9]
b_gyro_temperature_sensitivity_x = b_floats[10]
b_gyro_temperature_sensitivity_y = b_floats[11]
b_gyro_temperature_sensitivity_z = b_floats[12]
b_magn_ellipsoid_center_x = b_floats[13]
b_magn_ellipsoid_center_y = b_floats[14]
b_magn_ellipsoid_center_z = b_floats[15]
b_magn_ellipsoid_transform_0_0 = b_floats[16]
b_magn_ellipsoid_transform_0_1 = b_floats[17]
b_magn_ellipsoid_transform_0_2 = b_floats[18]
b_magn_ellipsoid_transform_1_0 = b_floats[19]
b_magn_ellipsoid_transform_1_1 = b_floats[20]
b_magn_ellipsoid_transform_1_2 = b_floats[21]
b_magn_ellipsoid_transform_2_0 = b_floats[22]
b_magn_ellipsoid_transform_2_1 = b_floats[23]
b_magn_ellipsoid_transform_2_2 = b_floats[24]
b_magn_working_temperature = b_floats[25]
b_magn_temperature_sensitivity_x = b_floats[26]
b_magn_temperature_sensitivity_y = b_floats[27]
b_magn_temperature_sensitivity_z = b_floats[28]

print('Board calibrated: '+('Yes' if b_brd_cal==1 else 'No'))
print('Board ID: '+b_id0+b_id1)

print('Accelerometer X min / max: '+repr(b_accel_min_x)+' / '+repr(b_accel_max_x))
print('Accelerometer Y min / max: '+repr(b_accel_min_y)+' / '+repr(b_accel_max_y))
print('Accelerometer Z min / max: '+repr(b_accel_min_z)+' / '+repr(b_accel_max_z))
print('Gyroscope offset X,Y,Z: '+repr(b_gyro_offset_x)+','+repr(b_gyro_offset_y)+','+repr(b_gyro_offset_z))
print('Gyroscope working temperature: '+repr(b_gyro_working_temperature))
print('Gyroscope temperature sensitivity X,Y,Z: '+repr(b_gyro_temperature_sensitivity_x)+','+
      repr(b_gyro_temperature_sensitivity_y)+','+repr(b_gyro_temperature_sensitivity_z))
print('Magnetometer ellipsoid center: {'+repr(b_magn_ellipsoid_center_x)+','+
      repr(magn_ellipsoid_center_y)+','+repr(magn_ellipsoid_center_z)+'}')
print('Magnetometer ellipsoid transform:\n{{'+repr(b_magn_ellipsoid_transform_0_0)+','+
      repr(b_magn_ellipsoid_transform_0_1)+','+repr(b_magn_ellipsoid_transform_0_2)+'},\n{'+
      repr(b_magn_ellipsoid_transform_1_0)+','+repr(b_magn_ellipsoid_transform_1_1)+','+
      repr(b_magn_ellipsoid_transform_1_2)+'},\n{'+repr(b_magn_ellipsoid_transform_2_0)+','+
      repr(b_magn_ellipsoid_transform_2_1)+','+repr(b_magn_ellipsoid_transform_2_2)+'}}')
print('Magnetometer working temperature: '+repr(b_magn_working_temperature))
print('Magnetometer temperature sensitivity: {'+repr(b_magn_temperature_sensitivity_x)+','+
      repr(b_magn_temperature_sensitivity_y)+','+repr(b_magn_temperature_sensitivity_z)+'}')

vari = raw_input("Do you wish to write these parameters to EEPROM? (yes/No): ")
print ("You entered " + vari+".")
if vari == "yes":
    print 'Saving these parameters to EEPROM. Do not unplug.'
    ser1.write('#Mcew')
    print 'Please wait for reply from serial:'
    print ser1.readline()
else:
    print 'Not saving to EEPROM.'


ser1.close
