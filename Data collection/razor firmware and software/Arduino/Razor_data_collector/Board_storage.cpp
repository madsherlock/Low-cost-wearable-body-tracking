// 
// 
// 

#include "Board_storage.h"
#include <avr/EEPROM.h>
//#include <EEPROM.h> //You can't include other libraries in a custom library (which this is)

#define BRD_EE_PUT(addr,strg)	eeprom_update_block((const void*)&strg, (void*)addr, sizeof(strg))
#define BRD_EE_GET(addr,strg)	eeprom_read_block((void*)&strg, (void*)addr, sizeof(strg))

#define BRD_STORAGE_ADDRESS	0
#define BRD_STORAGE_FLAG_LOADED	0
#define BRD_STORAGE_FLAG_CAL	1
#define BRD_STORAGE_FLAG_ID		2
#define BRD_STORAGE_FLAG_ACCEL	3
#define BRD_STORAGE_FLAG_GYRO	4
#define BRD_STORAGE_FLAG_MAGN	5
#define BRD_STORAGE_MASK_SPECIFICS_LOADED ((1 << BRD_STORAGE_FLAG_CAL) | (1 << BRD_STORAGE_FLAG_ID) | \
	(1 << BRD_STORAGE_FLAG_ACCEL) | (1 << BRD_STORAGE_FLAG_GYRO) | (1 << BRD_STORAGE_FLAG_MAGN))
#define BRD_STORAGE_MASK_FULLY_LOADED ((1 << BRD_STORAGE_FLAG_LOADED) | BRD_STORAGE_MASK_SPECIFICS_LOADED)





// Blocks until another byte is available on serial port
char readChar()
{
	while (Serial.available() < 1) {} // Block
	return Serial.read();
}

float readFloatBinary()
{
	union f_tag {
		byte b[4];
		float fval;
	} f;
	f.b[0] = readChar();
	f.b[1] = readChar();
	f.b[2] = readChar();
	f.b[3] = readChar();
	return f.fval;
}

void Board_storageClass::update_loaded_status() {
	if ((loaded & BRD_STORAGE_MASK_SPECIFICS_LOADED) == BRD_STORAGE_MASK_SPECIFICS_LOADED)
		loaded = BRD_STORAGE_MASK_FULLY_LOADED;
}

void Board_storageClass::load_from_eeprom() {
	BRD_EE_GET(BRD_STORAGE_ADDRESS, storage);
	//eeprom_read_block((void*)&storage, (void*)BRD_STORAGE_ADDRESS, sizeof(storage));
	//EEPROM.get(BRD_STORAGE_ADDRESS, storage);
	loaded = BRD_STORAGE_MASK_FULLY_LOADED;
}
void Board_storageClass::save_to_eeprom() {
	BRD_EE_PUT(BRD_STORAGE_ADDRESS, storage);
	//eeprom_update_block((const void*)&storage, (void*)BRD_STORAGE_ADDRESS, sizeof(storage));
	//EEPROM.put(BRD_STORAGE_ADDRESS, storage);
}

bool Board_storageClass::get_loaded_status() {
	return bitRead(loaded, BRD_STORAGE_FLAG_LOADED);
}
void Board_storageClass::clear_loaded_status() {
	loaded = 0;
}

bool Board_storageClass::get_calibration_status() {
	if (bitRead(loaded, BRD_STORAGE_FLAG_CAL))
		return storage.board_is_calibrated;
	return false;
}
void Board_storageClass::set_calibration_status(const bool calibration_status) {
	storage.board_is_calibrated = calibration_status;
	bitSet(loaded, BRD_STORAGE_FLAG_CAL);
	update_loaded_status();
}

void Board_storageClass::get_ID(char &id0, char &id1) {
	if (bitRead(loaded, BRD_STORAGE_FLAG_ID)) {
		id0 = storage.id0;
		id1 = storage.id1;
	}
}
void Board_storageClass::set_ID(const char id0, const char id1) {
	storage.id0 = id0;
	storage.id1 = id1;
	bitSet(loaded, BRD_STORAGE_FLAG_ID);
	update_loaded_status();
}

void Board_storageClass::get_accel(float min[3], float max[3]) {
	if (bitRead(loaded, BRD_STORAGE_FLAG_ACCEL)) {
		min[0] = storage.accel_x_min;
		max[0] = storage.accel_x_max;
		min[1] = storage.accel_y_min;
		max[1] = storage.accel_y_max;
		min[2] = storage.accel_z_min;
		max[2] = storage.accel_z_max;
	}
}
void Board_storageClass::set_accel(const float min[3], const float max[3]) {
	storage.accel_x_max = max[0];
	storage.accel_x_min = min[0];
	storage.accel_y_max = max[1];
	storage.accel_y_min = min[1];
	storage.accel_z_max = max[2];
	storage.accel_z_min = min[2];
	bitSet(loaded, BRD_STORAGE_FLAG_ACCEL);
	update_loaded_status();
}

void Board_storageClass::get_gyro(float offset[3], float &working_temperature, float temp_sensitivity[3]) {
	if (bitRead(loaded, BRD_STORAGE_FLAG_GYRO)) {
		offset[0] = storage.gyro_offset_x;
		offset[1] = storage.gyro_offset_y;
		offset[2] = storage.gyro_offset_z;
		working_temperature = storage.gyro_working_temperature;
		temp_sensitivity[0] = storage.gyro_temp_sensitivity_x;
		temp_sensitivity[1] = storage.gyro_temp_sensitivity_y;
		temp_sensitivity[2] = storage.gyro_temp_sensitivity_z;
	}
}
void Board_storageClass::set_gyro(const float offset[3], const float &working_temperature, const float temp_sensitivity[3]) {
	storage.gyro_offset_x = offset[0];
	storage.gyro_offset_y = offset[1];
	storage.gyro_offset_z = offset[2];
	storage.gyro_working_temperature = working_temperature;
	storage.gyro_temp_sensitivity_x = temp_sensitivity[0];
	storage.gyro_temp_sensitivity_y = temp_sensitivity[1];
	storage.gyro_temp_sensitivity_z = temp_sensitivity[2];
	bitSet(loaded, BRD_STORAGE_FLAG_GYRO);
	update_loaded_status();
}

void Board_storageClass::get_magn(float magn_ellipsoid_center[3], float magn_ellipsoid_transform[3][3],
	float &magn_working_temperature, float magn_temp_sensitivity[3]) {
	if (bitRead(loaded, BRD_STORAGE_FLAG_MAGN)) {
		magn_ellipsoid_center[0] = storage.magn_ellipsoid_center_x;
		magn_ellipsoid_center[1] = storage.magn_ellipsoid_center_y;
		magn_ellipsoid_center[2] = storage.magn_ellipsoid_center_z;
		magn_ellipsoid_transform[0][0] = storage.magn_ellipsoid_transform_0_0;
		magn_ellipsoid_transform[0][1] = storage.magn_ellipsoid_transform_0_1;
		magn_ellipsoid_transform[0][2] = storage.magn_ellipsoid_transform_0_2;
		magn_ellipsoid_transform[1][0] = storage.magn_ellipsoid_transform_1_0;
		magn_ellipsoid_transform[1][1] = storage.magn_ellipsoid_transform_1_1;
		magn_ellipsoid_transform[1][2] = storage.magn_ellipsoid_transform_1_2;
		magn_ellipsoid_transform[2][0] = storage.magn_ellipsoid_transform_2_0;
		magn_ellipsoid_transform[2][1] = storage.magn_ellipsoid_transform_2_1;
		magn_ellipsoid_transform[2][2] = storage.magn_ellipsoid_transform_2_2;
		magn_working_temperature = storage.magn_working_temperature;
		magn_temp_sensitivity[0] = storage.magn_temp_sensitivity_x;
		magn_temp_sensitivity[1] = storage.magn_temp_sensitivity_y;
		magn_temp_sensitivity[2] = storage.magn_temp_sensitivity_z;
	}
}
void Board_storageClass::set_magn(const float magn_ellipsoid_center[3], const float magn_ellipsoid_transform[3][3],
	const float &working_temperature, const float magn_temp_sensitivity[3]) {
	storage.magn_ellipsoid_center_x = magn_ellipsoid_center[0];
	storage.magn_ellipsoid_center_y = magn_ellipsoid_center[1];
	storage.magn_ellipsoid_center_z = magn_ellipsoid_center[2];
	storage.magn_ellipsoid_transform_0_0 = magn_ellipsoid_transform[0][0];
	storage.magn_ellipsoid_transform_0_1 = magn_ellipsoid_transform[0][1];
	storage.magn_ellipsoid_transform_0_2 = magn_ellipsoid_transform[0][2];
	storage.magn_ellipsoid_transform_1_0 = magn_ellipsoid_transform[1][0];
	storage.magn_ellipsoid_transform_1_1 = magn_ellipsoid_transform[1][1];
	storage.magn_ellipsoid_transform_1_2 = magn_ellipsoid_transform[1][2];
	storage.magn_ellipsoid_transform_2_0 = magn_ellipsoid_transform[2][0];
	storage.magn_ellipsoid_transform_2_1 = magn_ellipsoid_transform[2][1];
	storage.magn_ellipsoid_transform_2_2 = magn_ellipsoid_transform[2][2];
	storage.magn_working_temperature = working_temperature;
	storage.magn_temp_sensitivity_x = magn_temp_sensitivity[0];
	storage.magn_temp_sensitivity_y = magn_temp_sensitivity[1];
	storage.magn_temp_sensitivity_z = magn_temp_sensitivity[2];
	bitSet(loaded, BRD_STORAGE_FLAG_MAGN);
	update_loaded_status();
}

void Board_storageClass::get_all(bool &board_is_calibrated, char &id0, char &id1,
	float accel_min[3], float accel_max[3],
	float gyro_offset[3], float &gyro_working_temperature, float gyro_temp_sensitivity[3],
	float magn_ellipsoid_center[3], float magn_ellipsoid_transform[3][3],
	float &magn_working_temperature, float magn_temp_sensitivity[3]) {
	if (get_loaded_status()) {
		board_is_calibrated = get_calibration_status();
		get_ID(id0, id1);
		get_accel(accel_min, accel_max);
		get_gyro(gyro_offset, gyro_working_temperature, gyro_temp_sensitivity);
		get_magn(magn_ellipsoid_center, magn_ellipsoid_transform, magn_working_temperature, magn_temp_sensitivity);
	}
}
void Board_storageClass::set_all(const bool board_is_calibrated, const char id0, const char id1,
	const float accel_min[3], const float accel_max[3],
	const float gyro_offset[3], const float &gyro_working_temperature, const float gyro_temp_sensitivity[3],
	const float magn_ellipsoid_center[3], const float magn_ellipsoid_transform[3][3],
	const float &working_temperature, const float magn_temp_sensitivity[3],
	const bool save) {
	set_calibration_status(board_is_calibrated);
	set_ID(id0, id1);
	set_accel(accel_min,accel_max);
	set_gyro(gyro_offset, gyro_working_temperature, gyro_temp_sensitivity);
	set_magn(magn_ellipsoid_center, magn_ellipsoid_transform,working_temperature,magn_temp_sensitivity);
	if (save)
		save_to_eeprom();
}

void Board_storageClass::serial_read_calibration_status_binary() {
	char charReader = readChar();
	set_calibration_status(charReader != 0);
}
void Board_storageClass::serial_read_id_binary() {
	char id0 = readChar();
	char id1 = readChar();
	set_ID(id0, id1);
}
void Board_storageClass::serial_read_accel_binary() {
	float accel_min[3];
	float accel_max[3];
	for (uint8_t i = 0; i < 3; ++i)
		accel_min[i] = readFloatBinary();
	for (uint8_t i = 0; i < 3; ++i)
		accel_max[i] = readFloatBinary();
	set_accel(accel_min, accel_max);
}
void Board_storageClass::serial_read_gyro_binary() {
	float gyro_offset[3];
	float gyro_working_temperature;
	float gyro_temp_sensitivity[3];
	for (uint8_t i = 0; i < 3; ++i)
		gyro_offset[i] = readFloatBinary();
	gyro_working_temperature = readFloatBinary();
	for (uint8_t i = 0; i < 3; ++i)
		gyro_temp_sensitivity[i] = readFloatBinary();
	set_gyro(gyro_offset, gyro_working_temperature, gyro_temp_sensitivity);
}
void Board_storageClass::serial_read_magn_binary() {
	float magn_ellipsoid_center[3];
	float magn_ellipsoid_transform[3][3];
	float magn_working_temperature;
	float magn_temp_sensitivity[3];
	for (uint8_t i = 0; i < 3; ++i)
		magn_ellipsoid_center[i] = readFloatBinary();
	for (uint8_t i = 0; i < 3; ++i)
		for (uint8_t j = 0; j < 3; ++j)
			magn_ellipsoid_transform[i][j] = readFloatBinary();
	magn_working_temperature = readFloatBinary();
	for (uint8_t i = 0; i < 3; ++i)
		magn_temp_sensitivity[i] = readFloatBinary();
	set_magn(magn_ellipsoid_center, magn_ellipsoid_transform,magn_working_temperature,magn_temp_sensitivity);
}

void Board_storageClass::serial_read_all_binary() {
	serial_read_calibration_status_binary();
	serial_read_id_binary();
	serial_read_accel_binary();
	serial_read_gyro_binary();
	serial_read_magn_binary();
}

void Board_storageClass::serial_print_ID_text() {
	Serial.print(storage.id0); Serial.println(storage.id1);
}

void Board_storageClass::serial_print_ID_binary() {
	Serial.write((byte)storage.id0); Serial.write((byte)storage.id1);
}

void Board_storageClass::serial_print_all_text() {
	if (get_loaded_status()) {
		Serial.print("Board calibrated: ");
		Serial.println((storage.board_is_calibrated) ? ("Yes") : ("No"));
		Serial.print("Board ID: ");
		serial_print_ID_text();
		Serial.print("Accelerometer X min / max: ");
		Serial.print(storage.accel_x_min); Serial.print(" / "); Serial.println(storage.accel_x_max);
		Serial.print("Accelerometer Y min / max: ");
		Serial.print(storage.accel_y_min); Serial.print(" / "); Serial.println(storage.accel_y_max);
		Serial.print("Accelerometer Z min / max: ");
		Serial.print(storage.accel_z_min); Serial.print(" / "); Serial.println(storage.accel_z_max);
		Serial.print("Gyroscope offset X,Y,Z: ");
		Serial.print(storage.gyro_offset_x); Serial.print(',');
		Serial.print(storage.gyro_offset_y); Serial.print(',');
		Serial.println(storage.gyro_offset_z);
		Serial.print("Gyroscope working temperature: "); Serial.println(storage.gyro_working_temperature);
		Serial.print("Gyroscope temperature sensitivity X,Y,Z: ");
		Serial.print(storage.gyro_temp_sensitivity_x); Serial.print(',');
		Serial.print(storage.gyro_temp_sensitivity_y); Serial.print(',');
		Serial.println(storage.gyro_temp_sensitivity_z);
		Serial.print("Magnetometer ellipsoid center: {");
		Serial.print(storage.magn_ellipsoid_center_x); Serial.print(",");
		Serial.print(storage.magn_ellipsoid_center_y); Serial.print(",");
		Serial.print(storage.magn_ellipsoid_center_z); Serial.println("}");
		Serial.print("Magnetometer ellipsoid transform: {{");
		Serial.print(storage.magn_ellipsoid_transform_0_0); Serial.print(',');
		Serial.print(storage.magn_ellipsoid_transform_0_1); Serial.print(',');
		Serial.print(storage.magn_ellipsoid_transform_0_2); Serial.print("},{");
		Serial.print(storage.magn_ellipsoid_transform_1_0); Serial.print(',');
		Serial.print(storage.magn_ellipsoid_transform_1_1); Serial.print(',');
		Serial.print(storage.magn_ellipsoid_transform_1_2); Serial.print("},{");
		Serial.print(storage.magn_ellipsoid_transform_2_0); Serial.print(',');
		Serial.print(storage.magn_ellipsoid_transform_2_1); Serial.print(',');
		Serial.print(storage.magn_ellipsoid_transform_2_2); Serial.println("}}");
		Serial.print("Magnetometer working temperature: "); Serial.println(storage.magn_working_temperature);
		Serial.print("Magnetometer temperature sensitivity: {");
		Serial.print(storage.magn_temp_sensitivity_x); Serial.print(",");
		Serial.print(storage.magn_temp_sensitivity_y); Serial.print(",");
		Serial.print(storage.magn_temp_sensitivity_z); Serial.println("}");
	}
	else {
		Serial.println("Error: Board specifics not loaded.");
	}
}

void Board_storageClass::serial_print_all_binary() {
	if (get_loaded_status()) {
		//Write 119 bytes.
		Serial.write((byte)storage.board_is_calibrated);
		serial_print_ID_binary();
		Serial.write((byte*)(&(storage.accel_x_min)),4);
		Serial.write((byte*)(&(storage.accel_x_max)), 4);
		Serial.write((byte*)(&(storage.accel_y_min)), 4);
		Serial.write((byte*)(&(storage.accel_y_max)), 4);
		Serial.write((byte*)(&(storage.accel_z_min)), 4);
		Serial.write((byte*)(&(storage.accel_z_max)), 4);
		Serial.write((byte*)(&(storage.gyro_offset_x)), 4);
		Serial.write((byte*)(&(storage.gyro_offset_y)), 4);
		Serial.write((byte*)(&(storage.gyro_offset_z)), 4);
		Serial.write((byte*)(&(storage.gyro_working_temperature)), 4);
		Serial.write((byte*)(&(storage.gyro_temp_sensitivity_x)), 4);
		Serial.write((byte*)(&(storage.gyro_temp_sensitivity_y)), 4);
		Serial.write((byte*)(&(storage.gyro_temp_sensitivity_z)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_center_x)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_center_y)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_center_z)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_transform_0_0)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_transform_0_1)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_transform_0_2)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_transform_1_0)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_transform_1_1)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_transform_1_2)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_transform_2_0)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_transform_2_1)), 4);
		Serial.write((byte*)(&(storage.magn_ellipsoid_transform_2_2)), 4);
		Serial.write((byte*)(&(storage.magn_working_temperature)), 4);
		Serial.write((byte*)(&(storage.magn_temp_sensitivity_x)), 4);
		Serial.write((byte*)(&(storage.magn_temp_sensitivity_y)), 4);
		Serial.write((byte*)(&(storage.magn_temp_sensitivity_z)), 4);
	}
	else {
		//@TODO: Error output
	}
}






Board_storageClass Board_storage;

