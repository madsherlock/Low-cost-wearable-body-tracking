// Board_storage.h

#ifndef _BOARD_STORAGE_h
#define _BOARD_STORAGE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

struct Board_storageClass {
private:
	struct storage_t {
		bool board_is_calibrated;
		char id0;
		char id1;
		float accel_x_min, accel_x_max, accel_y_min, accel_y_max, accel_z_min, accel_z_max;
		float gyro_offset_x, gyro_offset_y, gyro_offset_z;
		float gyro_working_temperature;
		float gyro_temp_sensitivity_x, gyro_temp_sensitivity_y, gyro_temp_sensitivity_z;
		float magn_ellipsoid_center_x, magn_ellipsoid_center_y, magn_ellipsoid_center_z;
		float magn_ellipsoid_transform_0_0, magn_ellipsoid_transform_0_1, magn_ellipsoid_transform_0_2;
		float magn_ellipsoid_transform_1_0, magn_ellipsoid_transform_1_1, magn_ellipsoid_transform_1_2;
		float magn_ellipsoid_transform_2_0, magn_ellipsoid_transform_2_1, magn_ellipsoid_transform_2_2;
		float magn_working_temperature;
		float magn_temp_sensitivity_x, magn_temp_sensitivity_y, magn_temp_sensitivity_z;
	};
	storage_t storage;
	char loaded;
	void update_loaded_status();
public:
	Board_storageClass() :loaded(0) {}
	void load_from_eeprom();
	void save_to_eeprom();
	bool get_loaded_status();
	void clear_loaded_status();
	bool get_calibration_status();
	void set_calibration_status(const bool calibration_status = true);
	void get_ID(char &id0, char &id1);
	void set_ID(const char id0, const char id1);
	void get_accel(float min[3], float max[3]);
	void set_accel(const float min[3], const float max[3]);
	void get_gyro(float offset[3], float &working_temperature, float temp_sensitivity[3]);
	void set_gyro(const float offset[3], const float &working_temperature, const float temp_sensitivity[3]);
	void get_magn(float magn_ellipsoid_center[3], float magn_ellipsoid_transform[3][3],
		float &magn_working_temperature, float magn_temp_sensitivity[3]);
	void set_magn(const float magn_ellipsoid_center[3], const float magn_ellipsoid_transform[3][3],
		const float &working_temperature, const float magn_temp_sensitivity[3]);
	void get_all(bool &board_is_calibrated, char &id0, char &id1,
		float accel_min[3], float accel_max[3],
		float gyro_offset[3], float &gyro_working_temperature, float gyro_temp_sensitivity[3],
		float magn_ellipsoid_center[3], float magn_ellipsoid_transform[3][3],
		float &magn_working_temperature, float magn_temp_sensitivity[3]);
	void set_all(const bool board_is_calibrated, const char id0, const char id1,
		const float accel_min[3], const float accel_max[3],
		const float gyro_offset[3], const float &gyro_working_temperature, const float gyro_temp_sensitivity[3],
		const float magn_ellipsoid_center[3], const float magn_ellipsoid_transform[3][3],
		const float &magn_working_temperature, const float magn_temp_sensitivity[3],
		const bool save = false);
	void serial_read_calibration_status_binary();
	void serial_read_id_binary();
	void serial_read_accel_binary();
	void serial_read_gyro_binary();
	void serial_read_magn_binary();
	void serial_read_all_binary();
	void serial_print_ID_text();
	void serial_print_ID_binary();
	void serial_print_all_text();
	void serial_print_all_binary();
};


extern Board_storageClass Board_storage;



// Blocks until another byte is available on serial port
char readChar();

float readFloatBinary();

#endif

