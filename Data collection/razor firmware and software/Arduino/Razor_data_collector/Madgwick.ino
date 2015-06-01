#if M_FILTER == true
#define sampleFreq	(1000.0/OUTPUT__DATA_INTERVAL)

extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

// angular rate is in rad/s
// acceleration is in g
// flux is in G
volatile float gyro_rads[3];
volatile float accel_g[3];
volatile float magnetom_G[3];

extern volatile float G_Dt;

// Sensor readings should already be calibrated
void prepareMadgwick() {
	for (uint8_t i = 0; i < 3; ++i) {
		gyro_rads[i] = GYRO_SCALED_RAD(gyro[i]);
		accel_g[i] = -accel[i]*(1.0/GRAVITY);
		magnetom_G[i] = magnetom[i] * MAGN_GAIN;
	}
}

void MadgwickAHRSupdateIMU() {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	prepareMadgwick();
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gyro_rads[0] - q2 * gyro_rads[1] - q3 * gyro_rads[2]);
	qDot2 = 0.5f * (q0 * gyro_rads[0] + q2 * gyro_rads[2] - q3 * gyro_rads[1]);
	qDot3 = 0.5f * (q0 * gyro_rads[1] - q1 * gyro_rads[2] + q3 * gyro_rads[0]);
	qDot4 = 0.5f * (q0 * gyro_rads[2] + q1 * gyro_rads[1] - q2 * gyro_rads[0]);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((accel_g[0] == 0.0f) && (accel_g[1] == 0.0f) && (accel_g[2] == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(accel_g[0] * accel_g[0] + accel_g[1] * accel_g[1] + accel_g[2] * accel_g[2]);
		accel_g[0] *= recipNorm;
		accel_g[1] *= recipNorm;
		accel_g[2] *= recipNorm;
		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;
		// Gradient descent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * accel_g[0] + _4q0 * q1q1 - _2q1 * accel_g[1];
		s1 = _4q1 * q3q3 - _2q3 * accel_g[0] + 4.0f * q0q0 * q1 - _2q0 * accel_g[1] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * accel_g[2];
		s2 = 4.0f * q0q0 * q2 + _2q0 * accel_g[0] + _4q2 * q3q3 - _2q3 * accel_g[1] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * accel_g[2];
		s3 = 4.0f * q1q1 * q3 - _2q1 * accel_g[0] + 4.0f * q2q2 * q3 - _2q2 * accel_g[1];
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}
	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * G_Dt;
	q1 += qDot2 * G_Dt;
	q2 += qDot3 * G_Dt;
	q3 += qDot4 * G_Dt;
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void MadgwickAHRSupdate() {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((magnetom_G[0] == 0.0f) && (magnetom_G[1] == 0.0f) && (magnetom_G[2] == 0.0f)||(!use_mag)) {
		MadgwickAHRSupdateIMU();
		return;
	}
	prepareMadgwick();
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gyro_rads[0] - q2 * gyro_rads[1] - q3 * gyro_rads[2]);
	qDot2 = 0.5f * (q0 * gyro_rads[0] + q2 * gyro_rads[2] - q3 * gyro_rads[1]);
	qDot3 = 0.5f * (q0 * gyro_rads[1] - q1 * gyro_rads[2] + q3 * gyro_rads[0]);
	qDot4 = 0.5f * (q0 * gyro_rads[2] + q1 * gyro_rads[1] - q2 * gyro_rads[0]);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((accel_g[0] == 0.0f) && (accel_g[1] == 0.0f) && (accel_g[2] == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(accel_g[0] * accel_g[0] + accel_g[1] * accel_g[1] + accel_g[2] * accel_g[2]);
		accel_g[0] *= recipNorm;
		accel_g[1] *= recipNorm;
		accel_g[2] *= recipNorm;
		// Normalise magnetometer measurement
		recipNorm = invSqrt(magnetom_G[0] * magnetom_G[0] + magnetom_G[1] * magnetom_G[1] + magnetom_G[2] * magnetom_G[2]);
		magnetom_G[0] *= recipNorm;
		magnetom_G[1] *= recipNorm;
		magnetom_G[2] *= recipNorm;
		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * magnetom_G[0];
		_2q0my = 2.0f * q0 * magnetom_G[1];
		_2q0mz = 2.0f * q0 * magnetom_G[2];
		_2q1mx = 2.0f * q1 * magnetom_G[0];
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;
		// Reference direction of Earth's magnetic field
		hx = magnetom_G[0] * q0q0 - _2q0my * q3 + _2q0mz * q2 + magnetom_G[0] * q1q1 + _2q1 * magnetom_G[1] * q2 + _2q1 * magnetom_G[2] * q3 - magnetom_G[0] * q2q2 - magnetom_G[0] * q3q3;
		hy = _2q0mx * q3 + magnetom_G[1] * q0q0 - _2q0mz * q1 + _2q1mx * q2 - magnetom_G[1] * q1q1 + magnetom_G[1] * q2q2 + _2q2 * magnetom_G[2] * q3 - magnetom_G[1] * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + magnetom_G[2] * q0q0 + _2q1mx * q3 - magnetom_G[2] * q1q1 + _2q2 * magnetom_G[1] * q3 - magnetom_G[2] * q2q2 + magnetom_G[2] * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
		// Gradient descent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - accel_g[0]) + _2q1 * (2.0f * q0q1 + _2q2q3 - accel_g[1]) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magnetom_G[0]) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magnetom_G[1]) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - magnetom_G[2]);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - accel_g[0]) + _2q0 * (2.0f * q0q1 + _2q2q3 - accel_g[1]) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - accel_g[2]) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magnetom_G[0]) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magnetom_G[1]) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - magnetom_G[2]);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - accel_g[0]) + _2q3 * (2.0f * q0q1 + _2q2q3 - accel_g[1]) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - accel_g[2]) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magnetom_G[0]) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magnetom_G[1]) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - magnetom_G[2]);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - accel_g[0]) + _2q2 * (2.0f * q0q1 + _2q2q3 - accel_g[1]) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magnetom_G[0]) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magnetom_G[1]) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - magnetom_G[2]);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}
	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * G_Dt;
	q1 += qDot2 * G_Dt;
	q2 += qDot3 * G_Dt;
	q3 += qDot4 * G_Dt;
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}



#endif //M_FILTER == true