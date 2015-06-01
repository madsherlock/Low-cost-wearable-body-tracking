/* This file is part of the Razor AHRS Firmware */

// I2C code to read the sensors

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int) 0x68) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#if ARDUINO >= 100
#define WIRE_SEND(b) Wire.write((byte) b) 
#define WIRE_RECEIVE() Wire.read() 
#else
#define WIRE_SEND(b) Wire.send(b)
#define WIRE_RECEIVE() Wire.receive() 
#endif


void I2C_Init() {
	Wire.begin();
}

void Accel_Init() {
	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x2D);  // Power register
	WIRE_SEND(0x08);  // Measurement mode
	Wire.endTransmission();
	delay(5);
	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x31);  // Data format register
	WIRE_SEND(0x08);  // Set to full resolution
	Wire.endTransmission();
	delay(5);
	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x2C);  // Rate
	//WIRE_SEND(0x09);  // Set to 50Hz, normal operation
	WIRE_SEND(ACCEL_DR);  // normal op
	Wire.endTransmission();
	delay(5);
}

// Reads x, y and z accelerometer registers
void Read_Accel() {
	int i = 0;
	byte buff[6];
	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x32);  // Send address to read from
	Wire.endTransmission();
	Wire.beginTransmission(ACCEL_ADDRESS);
	Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
	while (Wire.available())  // ((Wire.available())&&(i<6))
	{
		buff[i] = WIRE_RECEIVE();  // Read one byte
		i++;
	}
	Wire.endTransmission();
	if (i == 6)  // All bytes received?
	{
		// No multiply by -1 for coordinate system transformation here, because of double negation:
		// We want the gravity vector, which is negated acceleration vector.
		accel[0] = (((int)buff[3]) << 8) | buff[2];  // X axis (internal sensor y axis)
		accel[1] = (((int)buff[1]) << 8) | buff[0];  // Y axis (internal sensor x axis)
		accel[2] = (((int)buff[5]) << 8) | buff[4];  // Z axis (internal sensor z axis)
	}
	else {
		num_accel_errors++;
		if (output_errors) Serial.println("!ERR: reading accelerometer");
	}
}


unsigned long magn_time = 0;
void Magn_Init() {
	if (magn_cont_mode) {
		Wire.beginTransmission(MAGN_ADDRESS);
		WIRE_SEND(0x02); //Mode register
		WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
		Wire.endTransmission();
		delay(5);
	}
	Wire.beginTransmission(MAGN_ADDRESS);
	WIRE_SEND(0x00); //Configuration Register A
	//WIRE_SEND(0b00011000);  // Set 50Hz (75 Hz)
	WIRE_SEND(MAGN_DR_75); //75 Hz
	Wire.endTransmission();
	if (magn_cont_mode) {
		delay(5);
	}
	if (!magn_cont_mode) {
		Wire.beginTransmission(MAGN_ADDRESS);
		WIRE_SEND(0x02); //Mode register
		WIRE_SEND(0x01); //Single measurement mode
		Wire.endTransmission();
		magn_time = millis();
		delay(5); //actually, 6 ms wait before data can be read...
	}
}

void Read_Magn() {
	if (!magn_cont_mode)
		if ((millis() - magn_time) < 6)
			return;
	int i = 0;
	byte buff[6];
	Wire.beginTransmission(MAGN_ADDRESS);
	WIRE_SEND(0x03);  // Send address to read from
	Wire.endTransmission();
	Wire.beginTransmission(MAGN_ADDRESS);
	Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
	while (Wire.available())  // ((Wire.available())&&(i<6))
	{
		buff[i] = WIRE_RECEIVE();  // Read one byte
		i++;
	}
	Wire.endTransmission();

	if (i == 6)  // All bytes received?
	{
		// 9DOF Razor IMU SEN-10125 using HMC5843 magnetometer
#if HW__VERSION_CODE == 10125
		// MSB byte first, then LSB; X, Y, Z
		magnetom[0] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // X axis (internal sensor -y axis)
		magnetom[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
		magnetom[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
		// 9DOF Razor IMU SEN-10736 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10736
		// MSB byte first, then LSB; Y and Z reversed: X, Z, Y
		magnetom[0] = -1 * ((((int)buff[4]) << 8) | buff[5]);  // X axis (internal sensor -y axis)
		magnetom[1] = -1 * ((((int)buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
		magnetom[2] = -1 * ((((int)buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
		// 9DOF Sensor Stick SEN-10183 and SEN-10321 using HMC5843 magnetometer
#elif (HW__VERSION_CODE == 10183) || (HW__VERSION_CODE == 10321)
		// MSB byte first, then LSB; X, Y, Z
		magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
		magnetom[1] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Y axis (internal sensor -y axis)
		magnetom[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
		// 9DOF Sensor Stick SEN-10724 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10724
		// MSB byte first, then LSB; Y and Z reversed: X, Z, Y
		magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
		magnetom[1] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
		magnetom[2] = -1 * ((((int)buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
#endif
	}
	else {
		num_magn_errors++;
		if (output_errors) Serial.println("!ERR: reading magnetometer");
	}
	if (!magn_cont_mode) {
		Wire.beginTransmission(MAGN_ADDRESS);
		WIRE_SEND(0x02); //Mode register
		WIRE_SEND(0x01); //Single measurement mode
		Wire.endTransmission(); //6 ms from now, data should be ready.
		magn_time = millis();
	}
}

void Gyro_Init() {
	// Power up reset defaults
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x3E); //PWR_MGM
	WIRE_SEND(0x80); //H_RESET
	Wire.endTransmission();
	delay(5);
	// Select full-scale range of the gyro sensors, set LP filter bandwidth
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x16);
	//WIRE_SEND(0x1B);  // DLPF_CFG = 3, FS_SEL = 3 (2000 deg/s)
	WIRE_SEND(0x18 | GYRO_DLPF);  // DLPF_CFG, FS_SEL = 3 (2000 deg/s)
	Wire.endTransmission();
	delay(5);
	//Set sample rate
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x15);
	//WIRE_SEND(0x0A);  //  SMPLRT_DIV = 10 (50Hz)
	WIRE_SEND(GYRO_DIVIDER);
	Wire.endTransmission();
	delay(5);
	// Set clock to PLL with z gyro reference
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x3E);
	WIRE_SEND(0x00);
	Wire.endTransmission();
	delay(5);
}

// Reads x, y and z gyroscope registers
void Read_Gyro() {
	int i = 0;
	byte buff[6];
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x1D);  // Sends address to read from
	Wire.endTransmission();
	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
	while (Wire.available())  // ((Wire.available())&&(i<6))
	{
		buff[i] = WIRE_RECEIVE();  // Read one byte
		i++;
	}
	Wire.endTransmission();
	if (i == 6)  // All bytes received?
	{
		gyro[0] = -1 * ((((int)buff[2]) << 8) | buff[3]);    // X axis (internal sensor -y axis)
		gyro[1] = -1 * ((((int)buff[0]) << 8) | buff[1]);    // Y axis (internal sensor -x axis)
		gyro[2] = -1 * ((((int)buff[4]) << 8) | buff[5]);    // Z axis (internal sensor -z axis)
	}
	else {
		num_gyro_errors++;
		if (output_errors) Serial.println("!ERR: reading gyroscope");
	}
}

void Read_Gyro_Thermometer() {
	int i = 0;
	byte buff[8];
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x1B);  // Sends address to read from
	Wire.endTransmission();
	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.requestFrom(GYRO_ADDRESS, 8);  // Request 6 bytes
	while (Wire.available())  // ((Wire.available())&&(i<6))
	{
		buff[i] = WIRE_RECEIVE();  // Read one byte
		i++;
	}
	Wire.endTransmission();
	if (i == 8)  // All bytes received?
	{
		temperature[0] = 35.0 + (((((float)((int16_t)((((int)buff[0] << 8) | ((int)buff[1]))))) + 13200.0))*GYRO_TEMPERATURE_SENSOR_SENSITIVITY);
		gyro[0] = -1 * ((((int)buff[4]) << 8) | buff[5]);    // X axis (internal sensor -y axis)
		gyro[1] = -1 * ((((int)buff[2]) << 8) | buff[3]);    // Y axis (internal sensor -x axis)
		gyro[2] = -1 * ((((int)buff[6]) << 8) | buff[7]);    // Z axis (internal sensor -z axis)
	}
	else {
		num_gyro_errors++;
		if (output_errors) Serial.println("!ERR: reading gyroscope");
	}
}
