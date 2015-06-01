/***************************************************************************************************************
* Razor data collection firmware v1.0.0 based on Razor AHRS Firmware v1.4.2 (see below)
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2015 Mikael Westermann
* mikaelwestermann@gmail.com
* Modifications to AHRS firmware include:
*   * No longer AHRS firmware - now it's only data collection firmware.
*   * Samling rate increased to 100 Hz.
*   * Several output and calibration functions added.
*   * Temperature measurements from gyroscope utilized.
* 
* Based on:
* Razor AHRS Firmware v1.4.2
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
* Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
*
*
* History:
*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
*
*   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik (david.zsolt.malik@gmail.com)
*     for new Sparkfun 9DOF Razor hardware (SEN-10125).
*
*   * Updated and extended by Peter Bartz (peter-bartz@gmx.de):
*     * v1.3.0
*       * Cleaned up, streamlined and restructured most of the code to make it more comprehensible.
*       * Added sensor calibration (improves precision and responsiveness a lot!).
*       * Added binary yaw/pitch/roll output.
*       * Added basic serial command interface to set output modes/calibrate sensors/synch stream/etc.
*       * Added support to synch automatically when using Rovering Networks Bluetooth modules (and compatible).
*       * Wrote new easier to use test program (using Processing).
*       * Added support for new version of "9DOF Razor IMU": SEN-10736.
*       --> The output of this code is not compatible with the older versions!
*       --> A Processing sketch to test the tracker is available.
*     * v1.3.1
*       * Initializing rotation matrix based on start-up sensor readings -> orientation OK right away.
*       * Adjusted gyro low-pass filter and output rate settings.
*     * v1.3.2
*       * Adapted code to work with new Arduino 1.0 (and older versions still).
*     * v1.3.3
*       * Improved synching.
*     * v1.4.0
*       * Added support for SparkFun "9DOF Sensor Stick" (versions SEN-10183, SEN-10321 and SEN-10724).
*     * v1.4.1
*       * Added output modes to read raw and/or calibrated sensor data in text or binary format.
*       * Added static magnetometer soft iron distortion compensation
*     * v1.4.2
*       * (No core firmware changes)
*
* TODOs:
*   * Allow optional use of EEPROM for storing and reading calibration values.
*   * Use self-test and temperature-compensation features of the sensors.
***************************************************************************************************************/

/*
  "9DOF Razor IMU" hardware versions: SEN-10125 and SEN-10736

  ATMega328@3.3V, 8MHz

  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10125
  HMC5883L : Magnetometer on SEN-10736
  ITG-3200 : Gyro

  Arduino IDE : Select board "Arduino Pro or Pro Mini (3.3v, 8Mhz) w/ATmega328"
  */

/*
  "9DOF Sensor Stick" hardware versions: SEN-10183, SEN-10321 and SEN-10724

  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10183 and SEN-10321
  HMC5883L : Magnetometer on SEN-10724
  ITG-3200 : Gyro
  */

/*
  Axis definition (differs from definition printed on the board!):
  X axis pointing forward (towards the short edge with the connector holes)
  Y axis pointing to the right
  and Z axis pointing down.

  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up

  Transformation order: first yaw then pitch then roll.
  */

/*
  Serial commands that the firmware understands:

  "#o<params>" - Set OUTPUT mode and parameters. The available options are:

  // Streaming output
  "#o0" - DISABLE continuous streaming output. Also see #f below.
  "#o1" - ENABLE continuous streaming output.

  // Angles output
  "#ob" - Output angles in BINARY format (yaw/pitch/roll as binary float, so one output frame
  is 3x4 = 12 bytes long).
  "#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
  followed by carriage return and line feed [\r\n]).

  // Sensor calibration
  "#oc" - Go to CALIBRATION output mode.
  "#on" - When in calibration mode, go on to calibrate NEXT sensor.

  // Sensor data output
  "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
  One frame consist of three lines - one for each sensor: acc, mag, gyr.
  "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
  One frame consist of three lines - one for each sensor: acc, mag, gyr.
  "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
  One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
  NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards.
  In fact it's too much and an output frame rate of 50Hz can not be maintained. #osbb.
  "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
  One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
  "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
  One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
  "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
  One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).

  // Error message output
  "#oe0" - Disable ERROR message output.
  "#oe1" - Enable ERROR message output.


  "#f" - Request one output frame - useful when continuous output is disabled and updates are
  required in larger intervals only. Though #f only requests one reply, replies are still
  bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.
  ¨
  //Added by Mikael Westermann:
  "#m" - Read sensors immediately and output afterwards. Similar to "#f", except not bound by internal period.
  "#t<b>" - b = 0 or 1: Toggles output one byte before other data, containing number of missed loops.
                        Only effective in binary output mode or one line text output mode.
			b = 2: Goes into gyro calibration mode, outputting text, until "#t0" or "#t1" is sent.
  "#e<b>" - b = 0 or 1. Toggles output temperature readings before other values.
  "#Moqt" - Output quaterion as text. (Only if M_FILTER is enabled)
  "#Moqb" - Output quaterion as binary. (Only if M_FILTER is enabled)
  "#Mma0" - Don't use magnetometer in filter. Recommended.
  "#Mma1" - Use magnetometer in filter. Not recommended (can't run at 100 Hz).

  "#Mcsb<binary data>" - Load all calibration parameters in <binary data> (103 bytes). Applied immediately.
  "#Mcscb<byte>" - Load calibration status in <byte>  (0 or 1).
  "#Mcsib<binary data>" - Load board ID in <binary data>  (2 bytes representing id0 and id1).
  "#Mcsab<binary data>" - Load accelerometer calibration parameters in <binary data> (6 floats of 4 bytes each)
						Format: min_x min_y min_z max_x max_y max_z
  "#Mcsgb<binary data>" - Load gyro calibration parameters in <binary data>  (7 floats of 4 bytes each)
						Format: offset_x offset_y offset_z working_temperature temp_sensitivity_x temp_sensitivity_y temp_sensitivity_z
  "#Mcsmb<binary data>" - Load magnetometer calibration parameters in <binary data>  (12 floats of 4 bytes each).
						Format: center_x center_y center_z t_0_0 t_0_1 t_0_2 t_1_0 t_1_1 t_1_2 t_2_0 t_2_1 t_2_2

  "#Mclt" - Output loaded calibration parameters as text.
	"#Mclit" - Output one line with board ID as text.
  "#Mclb" - Output loaded calibration parameters as binary data.

  "#Mcew" - Store all loaded calibration parameters in EEPROM. Reply with "OK" line (possibly after several milliseconds). Applied immediately.
  "#Mcer" - Read and load calibration parameters from EEPROM. Applied immediately.
  "#Mca" - Apply loaded calibration parameters (which can be viewed by sending "#Mclb" or "#Mclt") to this session.
  
  "#Mcoa<b>" - Enable (b='1') or disable (b='0') accelerometer offset and scaling compensation.
  "#Mcog1" - Enable gyroscope offset and scaling compensation.
  "#Mcog0" - Disable gyroscope offset and scaling compensation.
  "#Mcog3" - Enable gyroscope temperature compensation.
  "#Mcog2" - Disable gyroscope temperature compensation.
  "#Mcom1" - Enable magnetometer hard and soft iron distortion compensation.
  "#Mcom0" - Disable magnetometer hard and soft iron distortion compensation.
  "#Mcom3" - Enable magnetometer temperature compensation.
  "#Mcom2" - Disable magnetometer temperature compensation.


  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
  binary stream or to see if tracker is present and answering. The tracker will send
  "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
  x and y are two mandatory but arbitrary bytes that can be used to find out which request
  the answer belongs to.


  ("#C" and "#D" - Reserved for communication with optional Bluetooth module.)

  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.

  The status LED will be on if streaming output is enabled and off otherwise.

  Byte order of binary output is little-endian: least significant byte comes first.
  */



/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
#include "Board_storage.h"
#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)


// SPECIFIC BOARDS
//#define BRD__BIBL	0
//#define BRD__KJEN	1

//#define BRD__THIS	BRD__BIBL
//#define BRD__THIS	BRD__KJEN

//#include "Calibration.h"

#define DCM_FILTER	false
#define M_FILTER	false //12 ms loop time using magnetometer, 9 ms loop time without magnetometer.

// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 1000000

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 10  // in milliseconds

/** Accelerometer ADXL345 data rate
* 6.25 Hz: 0110 (40uA)
* 12.5 Hz: 0111 (55uA)
* 25 Hz: 1000 (65uA)
* 50 Hz: 1001 (100uA)
* 100 Hz: 1010 (145uA)
* 200 Hz: 1011 (145uA)
* 400 Hz: 1100 (145uA)
* 800 Hz: 1101 (145uA)
* 1600 Hz: 1110 (100uA ???)
* 3200 Hz: 1111 (145uA)
* Bandwidth = data rate / 2
*/
#define ACCEL_DR_6p25	0x06
#define ACCEL_DR_12p5	0x07
#define ACCEL_DR_25		0x08
#define ACCEL_DR_50		0x09
#define ACCEL_DR_100	0x0A
#define ACCEL_DR_200	0x0B
#define ACCEL_DR_400	0x0C
#define ACCEL_DR_800	0x0D
#define ACCEL_DR_1600	0x0E
#define ACCEL_DR_3200	0x0F
//Choose data rate below
#define ACCEL_DR		ACCEL_DR_100

/** Magnetometer HMC5883L output rate
* Min. continuous: 0.75 Hz
* Max. continuous: 75 Hz
* Single measurement mode: 160 Hz
* Measurement period from receiving command to data ready: 6 ms
* Turn-on Time ready for measurements: 50 ms
* Configuration register A:
* 0	MA1(0)	MA0(0)	DO2(1)	DO1(0)	DO0(0)	MS1(0)	MS0(0)
* MA1 to MA0 Select number of samples averaged per measurement output.
*	00 = 1, 01 = 2, 10 = 4, 11 = 8
*
* DO2 to DO0 Data Output Rate Bits.
* MS1 to MS0 Measurement Configuration Bits (bias or no)
*  Continuous measurement mode data rates:
*	DO2 DO1 DO0 Typical Data Output Rate (Hz)
*	0	0	0	0.75
*	0	0	1	1.5
*	0	1	0	3
*	0	1	1	7.5
*	1	0	0	15 (Default)
*	1	0	1	30
*	1	1	0	75
*	1	1	1	Reserved
*
*/
#define MAGN_DR_0p75	0x00
#define MAGN_DR_1p5		0b00000100
#define MAGN_DR_3		0b00001000
#define MAGN_DR_7p5		0b00001100
#define MAGN_DR_15		0b00010000
#define MAGN_DR_30		0b00010100
#define MAGN_DR_75		0b00011000
//Set to false if you want faster than 75 Hz (up to 160 Hz) data rate on magnetometer.
bool magn_cont_mode = false;

/** Gyroscope ITG-3200
* Low Pass Filter Bandwidth DLPF_CFG
*	0	256 Hz		Finternal 8kHz
*	1	188 Hz		Finternal 1kHz
*	2	98 Hz		-||-
*	3	42 Hz		...
*	4	20 Hz
*	5	10 Hz
*	6	5 Hz
*
* Fsample = Finternal/(divider+1) =>
* divider = Finternal/Fsample - 1
*/
#define GYRO_DLPF_256	0x00
#define GYRO_DLPF_188	0x01
#define GYRO_DLPF_98	0x02
#define GYRO_DLPF_42	0x03
#define GYRO_DLPF_20	0x04
#define GYRO_DLPF_10	0x05
#define GYRO_DLPF_5		0x06
//Choose low pass filter here
#define GYRO_DLPF	GYRO_DLPF_98
//Choose sample rate divider here
#define GYRO_DIVIDER	0x09 // 9 <==> 100Hz



// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#if DCM_FILTER == true
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#endif
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
#if M_FILTER == true
#define OUTPUT__MODE_QUATERNION 5
#endif
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
volatile int output_mode = (OUTPUT__MODE_SENSORS_RAW);
//volatile int output_mode = ((BRD__IS_CALIBRATED) ? (OUTPUT__MODE_SENSORS_CALIB) : (OUTPUT__MODE_SENSORS_RAW));
//volatile int output_mode = OUTPUT__MODE_QUATERNION;
volatile int output_format = OUTPUT__FORMAT_TEXT;

boolean one_line_output = true; //A,M,G
volatile boolean count_loops = false;
volatile boolean measure_temperature = true;
volatile boolean gyrocalibration = false;
#if M_FILTER == true
volatile boolean use_mag = false;
#endif


//Select which compensation strategies to apply
volatile boolean compensate_acc_offset_scaling	= true;
volatile boolean compensate_gyr_offset_scaling	= true;
volatile boolean compensate_gyr_temperature = true;
volatile boolean compensate_mag_hard_soft_iron = true;
volatile boolean compensate_mag_temperature = false;


// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON false  // true or false

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

// Bluetooth
// You can set this to true, if you have a Rovering Networks Bluetooth Module attached.
// The connect/disconnect message prefix of the module has to be set to "#".
// (Refer to manual, it can be set like this: SO,#)
// When using this, streaming output will only be enabled as long as we're connected. That way
// receiver and sender are synchronzed easily just by connecting/disconnecting.
// It is not necessary to set this! It just makes life easier when writing code for
// the receiving side. The Processing test sketch also works without setting this.
// NOTE: When using this, OUTPUT__STARTUP_STREAM_ON has no effect!
#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false




// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false


/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/










// Check if hardware version code is defined
#ifndef HW__VERSION_CODE
// Generate compile error
#error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.ino!
#endif

#include <Wire.h>

// Sensor calibration scale and offset values
//#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
//#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
//#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
//#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
//#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
//#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

//#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
//#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
//#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
//#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
//#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
//#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// Stuff
#define STATUS_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329251994329576923690768489)  // *pi/180
#define TO_DEG(x) (x * 57.295779513082320876798154814105)  // *180/pi

// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06956521739130434782608695652174 // Same gain 1/14.375 on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second
#define GYRO_TEMPERATURE_SENSOR_SENSITIVITY		((float)0.00357142857142857142857142857143) // 1/280 from datasheet

//Gain for magnetometer (HMC5883L)
#define MAGN_GAIN 0.00091743119266055045871559633027523 //  1/1090 LSBs/Gauss

#if DCM_FILTER == true
// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f
#endif

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

float temperature[1];

// Calibration storage variables
//float c_accel_min[3];
//float c_accel_max[3];
float c_accel_offset[3];
float c_accel_scale[3];
float c_gyro_offset[3];
float c_gyro_working_temperature;
float c_gyro_temperature_sensitivity[3];
float c_magn_ellipsoid_center[3];
float c_magn_ellipsoid_transform[3][3];
float c_magn_working_temperature;
float c_magn_temp_sensitivity[3];

#if DCM_FILTER == true
// DCM variables
float MAG_Heading;
float Accel_Vector[3] = { 0, 0, 0 }; // Store the acceleration in a vector
float Gyro_Vector[3] = { 0, 0, 0 }; // Store the gyros turn rate in a vector
float Omega_Vector[3] = { 0, 0, 0 }; // Corrected Gyro_Vector data
float Omega_P[3] = { 0, 0, 0 }; // Omega Proportional correction
float Omega_I[3] = { 0, 0, 0 }; // Omega Integrator
float Omega[3] = { 0, 0, 0 };
float errorRollPitch[3] = { 0, 0, 0 };
float errorYaw[3] = { 0, 0, 0 };
float DCM_Matrix[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
float Update_Matrix[3][3] = { { 0, 1, 2 }, { 3, 4, 5 }, { 6, 7, 8 } };
float Temporary_Matrix[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };


// Euler angles
float yaw;
float pitch;
float roll;
#endif

// DCM timing in the main loop
unsigned long timestamp;
#if (DCM_FILTER == true || M_FILTER == true)
unsigned long timestamp_old;
volatile float G_Dt; // Integration time for DCM algorithm
#endif

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
boolean output_force_single_on; //force immediate sensor reading!
volatile uint8_t missed_loops = 0;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

void read_sensors() {
	Read_Gyro_Thermometer(); // Read gyroscope
	Read_Accel(); // Read accelerometer
	Read_Magn(); // Read magnetometer
}

#if DCM_FILTER == true
// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
	float temp1[3];
	float temp2[3];
	float xAxis[] = { 1.0f, 0.0f, 0.0f };

	read_sensors();
	timestamp = millis();

	// GET PITCH
	// Using y-z-plane-component/x-component of gravity vector
	pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

	// GET ROLL
	// Compensate pitch of gravity vector 
	Vector_Cross_Product(temp1, accel, xAxis);
	Vector_Cross_Product(temp2, xAxis, temp1);
	// Normally using x-z-plane-component/y-component of compensated gravity vector
	// roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
	// Since we compensated for pitch, x-z-plane-component equals z-component:
	roll = atan2(temp2[1], temp2[2]);

	// GET YAW
	Compass_Heading();
	yaw = MAG_Heading;

	// Init rotation matrix
	init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}
#endif


#if M_FILTER == true
#define betaDef	((float)0.1)	//2(Kp)
volatile float beta = betaDef;
volatile float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0; // quaternion of sensor frame relative to auxiliary frame
#endif

void calibration_load_to_globals() {
	float c_accel_min[3];
	float c_accel_max[3];
	Board_storage.get_accel(c_accel_min, c_accel_max);
	for (uint8_t i = 0; i < 3; ++i) {
		c_accel_offset[i] = (c_accel_min[i] + c_accel_max[i]) / 2.0f;
		c_accel_scale[i] = (GRAVITY / (c_accel_max[i] - c_accel_offset[i]));
	}
	Board_storage.get_magn(c_magn_ellipsoid_center, c_magn_ellipsoid_transform,c_magn_working_temperature,c_magn_temp_sensitivity);
	Board_storage.get_gyro(c_gyro_offset, c_gyro_working_temperature, c_gyro_temperature_sensitivity);
}


// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
	float tdif;
	// Compensate accelerometer error
	if (compensate_acc_offset_scaling) {
		float accel_offset, accel_scale;
		for (uint8_t i = 0; i < 3; ++i) {
			//accel_offset = (c_accel_min[i] + c_accel_max[i]) / 2.0f;
			//accel_scale = (GRAVITY / (c_accel_max[i] - accel_offset));
			accel[i] = (accel[i] - c_accel_offset[i])*c_accel_scale[i];
		}
	}

	// Compensate magnetometer error
	// @todo	Make better magnetometer sensitivity scaling than just offset.
	//			All ellipsoid transform variables are probably also temperature sensitive.
	if (compensate_mag_temperature&&compensate_mag_hard_soft_iron) {
		tdif = (temperature[0] - (c_magn_working_temperature));
		for (uint8_t i = 0; i < 3; ++i)
			magnetom_tmp[i] = magnetom[i] - c_magn_ellipsoid_center[i] - tdif*c_magn_temp_sensitivity[i];
		Matrix_Vector_Multiply(c_magn_ellipsoid_transform, magnetom_tmp, magnetom);
	}
	else if(compensate_mag_hard_soft_iron) {
		for (uint8_t i = 0; i < 3; ++i)
			magnetom_tmp[i] = magnetom[i] - c_magn_ellipsoid_center[i];
		Matrix_Vector_Multiply(c_magn_ellipsoid_transform, magnetom_tmp, magnetom);
	}
	

	// Compensate gyroscope error
	if (compensate_gyr_temperature) {
		tdif = (temperature[0] - (c_gyro_working_temperature));
		for (uint8_t i = 0; i < 3; ++i)
			gyro[i] -= tdif*c_gyro_temperature_sensitivity[i];
	}
	if(compensate_gyr_offset_scaling)
		for (uint8_t i = 0; i < 3; ++i)
			gyro[i] -= c_gyro_offset[i];
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
	// Raw sensor values have to be read already, but no error compensation applied

	// Reset this calibration session?
	if (!reset_calibration_session_flag) return;

	// Reset acc and mag calibration variables
	for (int i = 0; i < 3; i++) {
		accel_min[i] = accel_max[i] = accel[i];
		magnetom_min[i] = magnetom_max[i] = magnetom[i];
	}

	// Reset gyro calibration variables
	gyro_num_samples = 0;  // Reset gyro calibration averaging
	gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;

	reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
	output_stream_on = true;
	digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
	output_stream_on = false;
	digitalWrite(STATUS_LED_PIN, LOW);
}



void setup()
{
	// Init serial output
	Serial.begin(OUTPUT__BAUD_RATE);

	// Init status LED
	pinMode(STATUS_LED_PIN, OUTPUT);
	digitalWrite(STATUS_LED_PIN, LOW);

	// Init sensors
	delay(50);  // Give sensors enough time to start
	I2C_Init();
	Accel_Init();
	Magn_Init();
	Gyro_Init();

	// Read sensors, init DCM algorithm
	delay(20);  // Give sensors enough time to collect data
#if DCM_FILTER == true
	reset_sensor_fusion();
#endif

	// Read board specifics from EEPROM
	Board_storage.load_from_eeprom();
	if (Board_storage.get_loaded_status()) {
		if (output_mode == (OUTPUT__MODE_SENSORS_RAW))
			output_mode = (OUTPUT__MODE_SENSORS_CALIB);
		calibration_load_to_globals();
	}


	// Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
	turn_output_stream_off();
#else
	turn_output_stream_on();
#endif
}

// Main loop
void loop()
{
	// Read incoming control messages
	if (Serial.available() >= 2) {
		if (Serial.read() == '#') { // Start of new control message
			int command = Serial.read(); // Commands
			if (command == 'f') // request one output _f_rame
				output_single_on = true;
			else if (command == 'm')
				output_force_single_on = true;
			else if (command == 't') {
				char toggler = readChar();
				if (toggler == '0')
					count_loops = false;
				else if (toggler == '1')
					count_loops = true;
			}
			else if (command == 'e') {
				char toggler = readChar();
				if (toggler == '0') {
					measure_temperature = false;
					gyrocalibration = false;
				}
				else if (toggler == '1') {
					measure_temperature = true;
					gyrocalibration = false;
				}
				else if (toggler == '2') {
					measure_temperature = true;
					gyrocalibration = true;
				}
			}
			else if (command == 'M') {
				char next = readChar();
				if(next=='c') {
					next=readChar();
					if(next=='s') {
						next=readChar();
						if(next=='b') {
							Board_storage.serial_read_all_binary();
							calibration_load_to_globals();
						}
						else if (next == 'c') {
							next = readChar();
							if (next == 'b') {
								Board_storage.serial_read_calibration_status_binary();
							}
						}
						else if (next == 'i') {
							next = readChar();
							if (next == 'b') {
								Board_storage.serial_read_id_binary();
							}
						}
						else if (next == 'a') {
							next = readChar();
							if (next == 'b') {
								float c_accel_min[3];
								float c_accel_max[3];
								Board_storage.serial_read_accel_binary();
								Board_storage.get_accel(c_accel_min, c_accel_max);
								for (uint8_t i = 0; i < 3; ++i) {
									c_accel_offset[i] = (c_accel_min[i] + c_accel_max[i]) / 2.0f;
									c_accel_scale[i] = (GRAVITY / (c_accel_max[i] - c_accel_offset[i]));
								}
							}
						}
						else if (next == 'g') {
							next = readChar();
							if (next == 'b') {
								Board_storage.serial_read_gyro_binary();
								Board_storage.get_gyro(c_gyro_offset, c_gyro_working_temperature, c_gyro_temperature_sensitivity);
							}
						}
						else if (next == 'm') {
							next = readChar();
							if (next == 'b') {
								Board_storage.serial_read_magn_binary();
								Board_storage.get_magn(c_magn_ellipsoid_center,
									c_magn_ellipsoid_transform,c_magn_working_temperature,c_magn_temp_sensitivity);
							}
						}
					}
					else if(next=='l') {
						next = readChar();
						if (next == 't') {
							Board_storage.serial_print_all_text();
						}
						else if (next == 'i') {
							next = readChar();
							if (next == 't') {
								Board_storage.serial_print_ID_text();
							}
							else if (next == 'b') {
								Board_storage.serial_print_ID_binary();
							}
						}
						else if (next == 'b') {
							Board_storage.serial_print_all_binary();
						}
					}
					else if(next=='e') {
						next=readChar();
						if(next == 'w') {
							Board_storage.save_to_eeprom();
							calibration_load_to_globals();
							Serial.println("OK");
						}
						else if(next=='r') {
							Board_storage.load_from_eeprom();
							calibration_load_to_globals();
						}
					}
					else if(next=='a') {
						calibration_load_to_globals();
					}
					else if(next=='o') {
						if(next=='a') {
							next = readChar();
							if (next == '0') {
								compensate_acc_offset_scaling = false;
							}
							else if(next=='1') {
								compensate_acc_offset_scaling = true;
							}
						}
						if(next=='g') {
							next = readChar();
							if (next == '0') {
								compensate_gyr_offset_scaling = false;
							}
							else if(next=='1') {
								compensate_gyr_offset_scaling = true;
							}
							else if (next == '2') {
								compensate_gyr_temperature = false;
							}
							else if (next == '3') {
								compensate_gyr_temperature = true;
							}
						}
						if(next=='m') {
							next = readChar();
							if (next == '0') {
								compensate_mag_hard_soft_iron = false;
							}
							else if(next=='1') {
								compensate_mag_hard_soft_iron = true;
							}
							else if (next == '2') {
								compensate_mag_temperature = false;
							}
							else if (next == '3') {
								compensate_mag_temperature = true;
							}
						}
					}
				}
#if M_FILTER == true
				else if (next == 'o') {
					next = readChar();
					if (next == 'q') {
						next = readChar();
						if (next == 't') {
							output_mode = OUTPUT__MODE_QUATERNION;
							output_format = OUTPUT__FORMAT_TEXT;
						}
						else if (next == 'b') {
							output_mode = OUTPUT__MODE_QUATERNION;
							output_format = OUTPUT__FORMAT_BINARY;
						}
					}
				}
				else if (next == 'm') {
					next = readChar();
					if (next == 'a') {
						next = readChar();
						if (next == '0') {
							use_mag = false;
						}
						else if (next == '1') {
							use_mag = true;
						}
					}
				}
#endif
			}
			else if (command == 's') { // _s_ynch request
				byte id[2]; // Read ID
				id[0] = readChar();
				id[1] = readChar();
				Serial.print("#SYNCH"); // Reply with synch message
				Serial.write(id, 2);
				Serial.println();
			}
			else if (command == 'o') { // Set _o_utput mode
				char output_param = readChar();
				if (output_param == 'n') { // Calibrate _n_ext sensor
					curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
					reset_calibration_session_flag = true;
				}
#if DCM_FILTER == true
				else if (output_param == 't')  { // Output angles as _t_ext
					output_mode = OUTPUT__MODE_ANGLES;
					output_format = OUTPUT__FORMAT_TEXT;
				}
				else if (output_param == 'b')  { // Output angles in _b_inary format
					output_mode = OUTPUT__MODE_ANGLES;
					output_format = OUTPUT__FORMAT_BINARY;
				}
#endif
				else if (output_param == 'c')  { // Go to _c_alibration mode
					output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
					reset_calibration_session_flag = true;
				}
				else if (output_param == 's') { // Output _s_ensor values
					char values_param = readChar();
					char format_param = readChar();
					if (values_param == 'r')  // Output _r_aw sensor values
						output_mode = OUTPUT__MODE_SENSORS_RAW;
					else if (values_param == 'c')  // Output _c_alibrated sensor values
						output_mode = OUTPUT__MODE_SENSORS_CALIB;
					else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
						output_mode = OUTPUT__MODE_SENSORS_BOTH;
					if (format_param == 't') // Output values as _t_text
						output_format = OUTPUT__FORMAT_TEXT;
					else if (format_param == 'b') // Output values in _b_inary format
						output_format = OUTPUT__FORMAT_BINARY;
				}
				else if (output_param == '0') { // Disable continuous streaming output
					turn_output_stream_off();
					reset_calibration_session_flag = true;
				}
				else if (output_param == '1')  { // Enable continuous streaming output
					reset_calibration_session_flag = true;
					turn_output_stream_on();
				}
				else if (output_param == 'e')  { // _e_rror output settings
					char error_param = readChar();
					if (error_param == '0') output_errors = false;
					else if (error_param == '1') output_errors = true;
					else if (error_param == 'c') { // get error count
						Serial.print("#AMG-ERR:");
						Serial.print(num_accel_errors); Serial.print(",");
						Serial.print(num_magn_errors); Serial.print(",");
						Serial.println(num_gyro_errors);
					}
				}
			}
#if OUTPUT__HAS_RN_BLUETOOTH == true
			// Read messages from bluetooth module
			// For this to work, the connect/disconnect message prefix of the module has to be set to "#".
			else if (command == 'C') // Bluetooth "#CONNECT" message (does the same as "#o1")
				turn_output_stream_on();
			else if (command == 'D') // Bluetooth "#DISCONNECT" message (does the same as "#o0")
				turn_output_stream_off();
#endif // OUTPUT__HAS_RN_BLUETOOTH == true
		}
		else
		{
		} // Skip character
	}

	if (output_force_single_on) {
#if M_FILTER == true
		if (output_mode == OUTPUT__MODE_QUATERNION) {
			timestamp_old = timestamp;
			timestamp = millis();
			if (timestamp > timestamp_old)
				G_Dt = (float)(timestamp - timestamp_old) / 1000.0f;
			else G_Dt = 0;
			read_sensors();
			output_quaternion();
		}
		else {
#endif
			read_sensors();
			if (gyrocalibration)
				output_gyro_text();
			else if (one_line_output)
				output_sensors_oneline();
			else
				output_sensors();
			output_force_single_on = false;
#if M_FILTER == true
		}
#endif
	}
	// Time to read the sensors again?
	if ((millis() - timestamp) >= OUTPUT__DATA_INTERVAL) {
#if (DCM_FILTER == true ||M_FILTER == true)
		timestamp_old = timestamp;
#endif
		timestamp = millis();
#if (DCM_FILTER == true || M_FILTER == true)
		if (timestamp > timestamp_old)
			G_Dt = (float)(timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
		else G_Dt = 0;
#endif

		if (count_loops) ++missed_loops;

		// Update sensor readings
		read_sensors();

		if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS) { // We're in calibration mode
			check_reset_calibration_session();  // Check if this session needs a reset
			if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
		}
#if DCM_FILTER == true
		else if (output_mode == OUTPUT__MODE_ANGLES) { // Output angles
			compensate_sensor_errors(); // Apply sensor calibration
			// Run DCM algorithm
			Compass_Heading(); // Calculate magnetic heading
			Matrix_update();
			Normalize();
			Drift_correction();
			Euler_angles();
			if (output_stream_on || output_single_on) output_angles();
		}
#endif
#if M_FILTER == true
		else if (output_mode == OUTPUT__MODE_QUATERNION) {
			compensate_sensor_errors();
			MadgwickAHRSupdate();
			output_quaternion();
		}
#endif
		else  { // Output sensor values
			if (output_stream_on || output_single_on) {
				if (gyrocalibration) {
					output_gyro_text();
				}
				else {
					if (one_line_output) {
						output_sensors_oneline();
					}
					else {
						output_sensors();
					}
				}
			}
		}

		output_single_on = false;

#if DEBUG__PRINT_LOOP_TIME == true
		Serial.print("loop time (ms) = ");
		Serial.println(millis() - timestamp);
#endif
	}
#if DEBUG__PRINT_LOOP_TIME == true
	else
	{
		Serial.println("waiting...");
	}
#endif
}
