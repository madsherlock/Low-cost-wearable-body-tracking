/* This file is part of the Razor AHRS Firmware */

#if DCM_FILTER == true
// Output angles: yaw, pitch, roll
void output_angles()
{
	if (output_format == OUTPUT__FORMAT_BINARY)
	{
		float ypr[3];
		ypr[0] = TO_DEG(yaw);
		ypr[1] = TO_DEG(pitch);
		ypr[2] = TO_DEG(roll);
		Serial.write((byte*)ypr, 12);  // No new-line
	}
	else if (output_format == OUTPUT__FORMAT_TEXT)
	{
		Serial.print("#YPR=");
		Serial.print(TO_DEG(yaw)); Serial.print(",");
		Serial.print(TO_DEG(pitch)); Serial.print(",");
		Serial.print(TO_DEG(roll)); Serial.println();
	}
}
#endif

#if M_FILTER == true
void output_quaternion() {
	/*
	if (output_format == OUTPUT__FORMAT_BINARY) {
		float quaternionlocal[4];
		quaternionlocal[0] = q0;
		quaternionlocal[1] = q1;
		quaternionlocal[2] = q2;
		quaternionlocal[3] = q3;
		Serial.write((byte*)quaternionlocal, 16);  // No new-line
	}
	else if (output_format == OUTPUT__FORMAT_TEXT) {
		Serial.print(q0); Serial.print(",");
		Serial.print(q1); Serial.print(",");
		Serial.print(q2); Serial.print(",");
		Serial.println(q3);
	}
	*/
}
#endif

void output_calibration(int calibration_sensor)
{
	if (calibration_sensor == 0)  // Accelerometer
	{
		// Output MIN/MAX values
		Serial.print("accel x,y,z (min/max) = ");
		for (int i = 0; i < 3; i++) {
			if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
			if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
			Serial.print(accel_min[i]);
			Serial.print("/");
			Serial.print(accel_max[i]);
			if (i < 2) Serial.print("  ");
			else Serial.println();
		}
	}
	else if (calibration_sensor == 1)  // Magnetometer
	{
		// Output MIN/MAX values
		Serial.print("magn x,y,z (min/max) = ");
		for (int i = 0; i < 3; i++) {
			if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
			if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
			Serial.print(magnetom_min[i]);
			Serial.print("/");
			Serial.print(magnetom_max[i]);
			if (i < 2) Serial.print("  ");
			else Serial.println();
		}
	}
	else if (calibration_sensor == 2)  // Gyroscope
	{
		// Average gyro values
		for (int i = 0; i < 3; i++)
			gyro_average[i] += gyro[i];
		gyro_num_samples++;

		// Output current and averaged gyroscope values
		Serial.print("gyro x,y,z (current/average) = ");
		for (int i = 0; i < 3; i++) {
			Serial.print(gyro[i]);
			Serial.print("/");
			Serial.print(gyro_average[i] / (float)gyro_num_samples);
			if (i < 2) Serial.print("  ");
			else Serial.println();
		}
	}
}

void output_sensors_text(char raw_or_calibrated)
{
	Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
	Serial.print(accel[0]); Serial.print(",");
	Serial.print(accel[1]); Serial.print(",");
	Serial.print(accel[2]); Serial.println();

	Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
	Serial.print(magnetom[0]); Serial.print(",");
	Serial.print(magnetom[1]); Serial.print(",");
	Serial.print(magnetom[2]); Serial.println();

	Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
	Serial.print(gyro[0]); Serial.print(",");
	Serial.print(gyro[1]); Serial.print(",");
	Serial.print(gyro[2]); Serial.println();
}


void output_gyro_text() {
	if (measure_temperature)
		Serial.print(temperature[0]); Serial.print(",");
	Serial.print(gyro[0]); Serial.print(",");
	Serial.print(gyro[1]); Serial.print(",");
	Serial.print(gyro[2]); Serial.println();
}

void output_sensors_binary() {
	if (count_loops) {
		Serial.write((missed_loops - 1));
		missed_loops = 0;
	}
	if (measure_temperature)
		Serial.write((byte*)temperature, 4);
	Serial.write((byte*)accel, 12);
	Serial.write((byte*)magnetom, 12);
	Serial.write((byte*)gyro, 12);

}

void output_sensors_oneline() {
	if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
		compensate_sensor_errors();
	if (output_format == OUTPUT__FORMAT_BINARY) {
		output_sensors_binary();
	}
	else {
		if (count_loops) {
			Serial.print((missed_loops - 1)); Serial.print(",");
			missed_loops = 0;
		}
		if (measure_temperature) {
			Serial.print(temperature[0]); Serial.print(",");
		}
		for (uint8_t i = 0; i < 3; ++i) { Serial.print(accel[i]); Serial.print(","); }
		for (uint8_t i = 0; i < 3; ++i) { Serial.print(magnetom[i]); Serial.print(","); }
		Serial.print(gyro[0]); Serial.print(",");
		Serial.print(gyro[1]); Serial.print(",");
		Serial.print(gyro[2]); Serial.println();
	}
}

void output_sensors()
{
	if (output_mode == OUTPUT__MODE_SENSORS_RAW) {
		if (output_format == OUTPUT__FORMAT_BINARY)
			output_sensors_binary();
		else if (output_format == OUTPUT__FORMAT_TEXT)
			output_sensors_text('R');
	}
	else if (output_mode == OUTPUT__MODE_SENSORS_CALIB) {
		compensate_sensor_errors(); // Apply sensor calibration
		if (output_format == OUTPUT__FORMAT_BINARY)
			output_sensors_binary();
		else if (output_format == OUTPUT__FORMAT_TEXT)
			output_sensors_text('C');
	}
	else if (output_mode == OUTPUT__MODE_SENSORS_BOTH) {
		if (output_format == OUTPUT__FORMAT_BINARY) {
			output_sensors_binary();
			compensate_sensor_errors();
			output_sensors_binary();
		}
		else if (output_format == OUTPUT__FORMAT_TEXT) {
			output_sensors_text('R');
			compensate_sensor_errors();
			output_sensors_text('C');
		}
	}
}

