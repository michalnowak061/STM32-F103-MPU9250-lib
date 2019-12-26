/*
 * mpu9250.c
 *
 *  Created on: 04.07.2019
 *  Author: mnowak5
 */

#include "mpu9250.h"

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Accelerometer_Configuration(I2C_HandleTypeDef *I2Cx,
													   struct MPU9250 *DataStructure,
													   MPU9250_Acce_range Range) {

	uint8_t Byte_temp = 0x00;

	/* Case 1: Set accelerometer sensitivity range */
	Byte_temp = Range << 3;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Device_addres, MPU9250_ACCEL_CONFIG, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Accelerometer_Config_FAIL;
	}

	/* Case 2: Set accelerometer low pass filter cut-off frequency */
	/*
	Byte_temp = 0x0E;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Device_addres, MPU9250_ACCEL_CONFIG_2, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Accelerometer_Config_FAIL;
	}
	*/

	/* Case 3: Save configuration to data structure */
	if(      Range == MPU9250_Acce_2G )     DataStructure->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_2G;
	else if( Range == MPU9250_Acce_4G )		DataStructure->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_4G;
	else if( Range == MPU9250_Acce_8G )		DataStructure->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_8G;
	else if( Range == MPU9250_Acce_16G )	DataStructure->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_16G;

	DataStructure->Accelerometer_X_offset = 0;
	DataStructure->Accelerometer_Y_offset = 0;
	DataStructure->Accelerometer_Z_offset = 0;

	return MPU9250_Accelerometer_Config_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Gyroscope_Configuration(I2C_HandleTypeDef *I2Cx,
												   struct MPU9250 *DataStructure,
												   MPU9250_Gyro_range Range) {

	uint8_t Byte_temp = 0x00;


	/* Case 1: Set gyroscope sensitivity range */
	Byte_temp = Range << 3;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Device_addres, MPU9250_GYRO_CONFIG, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Gyroscope_Config_FAIL;
	}

	/* Case 2: Set gyroscope low pass filter cut-off frequency */
	Byte_temp = 0x0E;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Device_addres, MPU9250_CONFIG, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Gyroscope_Config_FAIL;
	}

	/* Case 3: Save configuration to data structure */
	if(      Range == MPU9250_Gyro_250s )   DataStructure->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_250s;
	else if( Range == MPU9250_Gyro_500s )	DataStructure->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_500s;
	else if( Range == MPU9250_Gyro_1000s )	DataStructure->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_1000s;
	else if( Range == MPU9250_Gyro_2000s )	DataStructure->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_2000s;

	DataStructure->Gyroscope_X_offset = 0;
	DataStructure->Gyroscope_Y_offset = 0;
	DataStructure->Gyroscope_Z_offset = 0;

	return MPU9250_Gyroscope_Config_OK;

}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Magnetometer_Configuration(I2C_HandleTypeDef *I2Cx,
												      struct MPU9250 *DataStructure) {


	uint8_t Byte_temp = 0x00;
	uint8_t Bytes_temp[3] = {0};

	DataStructure->Magnetometer_addres = 0x0C << 1;
	DataStructure->Magnetometer_sesitivity_factor = 0.1499; /* 4912/32768 */

	// Case 2: Disable the I2C master interface
	Byte_temp = 0x00;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Device_addres, MPU9250_USER_CTRL, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	// Case 3: Enable the bypass multiplexer
	Byte_temp = 0x02;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Device_addres, MPU9250_INT_PIN_CFG, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	// Case 1: Is device connected ?
	if( HAL_I2C_IsDeviceReady(I2Cx, DataStructure->Magnetometer_addres, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	// Case 2: Who am i test
	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Magnetometer_addres, AK9863_WIA, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Init_FAIL;
	}

	if( Byte_temp != 0x48 ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Init_FAIL;
	}

	// Case 4: Setup to fuse ROM access mode and 16-bit output
	Byte_temp = 0x1F;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Magnetometer_addres, AK9863_CNTL1, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	HAL_Delay(100);

	// Case 5: Read from the fuse ROM sensitivity adjustment values
	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Magnetometer_addres, AK9863_ASAX | 0x80, 1, Bytes_temp, 3, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	DataStructure->Magnetometer_ASAX = ( ( (Bytes_temp[0] - 128) * 0.5 ) / 128 ) + 1;
	DataStructure->Magnetometer_ASAY = ( ( (Bytes_temp[1] - 128) * 0.5 ) / 128 ) + 1;
	DataStructure->Magnetometer_ASAZ = ( ( (Bytes_temp[2] - 128) * 0.5 ) / 128 ) + 1;

	// Case 6: Reset to power down mode
	Byte_temp = 0x00;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Magnetometer_addres, AK9863_CNTL1, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	// Case 7: Enable continuous mode 2 and 16-bit output
	Byte_temp = 0x16; // 0x16

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Magnetometer_addres, AK9863_CNTL1, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	HAL_Delay(100);

	/* Default variables value */
	DataStructure->Magnetometer_X_scale = 1;
	DataStructure->Magnetometer_Y_scale = 1;
	DataStructure->Magnetometer_Z_scale = 1;

	DataStructure->Magnetometer_X_offset = 0;
	DataStructure->Magnetometer_Y_offset = 0;
	DataStructure->Magnetometer_Z_offset = 0;

	return MPU9250_Magnetometer_Config_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Init(I2C_HandleTypeDef *I2Cx,
								struct MPU9250 *DataStructure,
								MPU9250_Device_number Number,
								MPU9250_Acce_range Acce_range,
								MPU9250_Gyro_range Gyro_range) {

	uint8_t Byte_temp = 0x00;

	DataStructure->Device_number = Number;
	DataStructure->Device_addres = (0x68 | DataStructure->Device_number) << 1;

	/* Case 1: Is device connected ? */
	if( HAL_I2C_IsDeviceReady(I2Cx, DataStructure->Device_addres, 1, 1000) != HAL_OK ) {

		return MPU9250_Init_FAIL;
	}

	/* Case 2: Who am i test */
	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_WHO_AM_I, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Init_FAIL;
	}

	if( Byte_temp != 0x71 ) {

		return MPU9250_Init_FAIL;
	}

	/* Case 3: Wake up */
	Byte_temp = 0x01;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Device_addres, MPU9250_PWR_MGMT_1, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Init_FAIL;
	}

	/* Case 4: Accelerometer configuration */
	if( MPU9250_Accelerometer_Configuration(I2Cx, DataStructure, Acce_range) != MPU9250_Accelerometer_Config_OK ) {

		return MPU9250_Accelerometer_Config_FAIL;
	}

	/* Case 5: Gyroscope configuration */
	if( MPU9250_Gyroscope_Configuration(I2Cx, DataStructure, Gyro_range) != MPU9250_Gyroscope_Config_OK ) {

		return MPU9250_Gyroscope_Config_FAIL;
	}

	/* Case 6: Magnetometer configuration */
	if( MPU9250_Magnetometer_Configuration(I2Cx, DataStructure) != MPU9250_Magnetometer_Config_OK ) {

		return MPU9250_Magnetometer_Config_FAIL;
	}

	/* Case 7: Default values of variables */

	quaternion_init( &(DataStructure->Gyroscope_quaternion) );
	euler_init( &(DataStructure->Gyroscope_euler) );

	quaternion_init( &(DataStructure->Madgwick_quaternion) );
	euler_init( &(DataStructure->Madgwick_euler) );

	return MPU9250_Init_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Accelerometer(I2C_HandleTypeDef *I2Cx,
											  struct MPU9250 *DataStructure) {

	uint8_t Bytes_temp[6] = {0x00};

	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_ACCEL_XOUT_H, 1, Bytes_temp , 6, 1000) != HAL_OK ) {

		return MPU9250_Read_Accelerometer_FAIL;
	}

	DataStructure->Accelerometer_X = ( Bytes_temp[0] << 8 | Bytes_temp[1] ) - DataStructure->Accelerometer_X_offset;
	DataStructure->Accelerometer_Y = ( Bytes_temp[2] << 8 | Bytes_temp[3] ) - DataStructure->Accelerometer_Y_offset;
	DataStructure->Accelerometer_Z = ( Bytes_temp[4] << 8 | Bytes_temp[5] ) - DataStructure->Accelerometer_Z_offset;

	/* Case x: Calculate g-force values for XYZ axis */
	DataStructure->Accelerometer_X_g = (float)(DataStructure->Accelerometer_X) / DataStructure->Accelerometer_sensitivity_factor;
	DataStructure->Accelerometer_Y_g = (float)(DataStructure->Accelerometer_Y) / DataStructure->Accelerometer_sensitivity_factor;
	DataStructure->Accelerometer_Z_g = (float)(DataStructure->Accelerometer_Z) / DataStructure->Accelerometer_sensitivity_factor;

	return MPU9250_Read_Accelerometer_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Gyroscope(I2C_HandleTypeDef *I2Cx,
										  struct MPU9250 *DataStructure) {

	uint8_t Bytes_temp[6] = { 0x00 };

	if (HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_GYRO_XOUT_H, 1, Bytes_temp, 6, 1000) != HAL_OK) {

		return MPU9250_Read_Gyroscope_FAIL;
	}

	DataStructure->Gyroscope_X = ( Bytes_temp[0] << 8 | Bytes_temp[1] ) - DataStructure->Gyroscope_X_offset;
	DataStructure->Gyroscope_Y = ( Bytes_temp[2] << 8 | Bytes_temp[3] ) - DataStructure->Gyroscope_Y_offset;
	DataStructure->Gyroscope_Z = ( Bytes_temp[4] << 8 | Bytes_temp[5] ) - DataStructure->Gyroscope_Z_offset;

	/* Case x: Calculate dgs/s values for XYZ axis */
	DataStructure->Gyroscope_X_dgs =  (double)(DataStructure->Gyroscope_X) / DataStructure->Gyroscope_sensitivity_factor;
	DataStructure->Gyroscope_Y_dgs =  (double)(DataStructure->Gyroscope_Y) / DataStructure->Gyroscope_sensitivity_factor;
	DataStructure->Gyroscope_Z_dgs =  (double)(DataStructure->Gyroscope_Z) / DataStructure->Gyroscope_sensitivity_factor;

	return MPU9250_Read_Gyroscope_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Magnetometer(I2C_HandleTypeDef *I2Cx,
										     struct MPU9250 *DataStructure) {

	uint8_t Bytes_temp[8] = { 0x00 };

	/* Case x: Read measured values from registers */
	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Magnetometer_addres, AK9863_ST1, 1, Bytes_temp, 8, 1000) != HAL_OK ) {

		return MPU9250_Read_Magnetometer_FAIL;
	}

	if( Bytes_temp[0] & 0x00 ) {

		return MPU9250_Read_Magnetometer_FAIL;
	}

	DataStructure->Magnetometer_X = ( ( Bytes_temp[2] << 8 | Bytes_temp[1] ) - DataStructure->Magnetometer_X_offset );
	DataStructure->Magnetometer_Y = ( ( Bytes_temp[4] << 8 | Bytes_temp[3] ) - DataStructure->Magnetometer_Y_offset );
	DataStructure->Magnetometer_Z = ( ( Bytes_temp[6] << 8 | Bytes_temp[5] ) - DataStructure->Magnetometer_Z_offset );

	/* Case x: Calculate uT (micro Tesla) value for XYZ axis */
	DataStructure->Magnetometer_X_uT = DataStructure->Magnetometer_X * DataStructure->Magnetometer_ASAX * DataStructure->Magnetometer_sesitivity_factor * DataStructure->Magnetometer_X_scale;
	DataStructure->Magnetometer_Y_uT = DataStructure->Magnetometer_Y * DataStructure->Magnetometer_ASAY * DataStructure->Magnetometer_sesitivity_factor * DataStructure->Magnetometer_Y_scale;
	DataStructure->Magnetometer_Z_uT = DataStructure->Magnetometer_Z * DataStructure->Magnetometer_ASAZ * DataStructure->Magnetometer_sesitivity_factor * DataStructure->Magnetometer_Z_scale;

	float a = DataStructure->Magnetometer_X_uT;
	float b = DataStructure->Magnetometer_Y_uT;
	float c = DataStructure->Magnetometer_Z_uT;

	DataStructure->Magnetometer_X_uT = b;
	DataStructure->Magnetometer_Y_uT = a;
	DataStructure->Magnetometer_Z_uT = -c;

	return MPU9250_Read_Magnetometer_OK;
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Calibration_Acce(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	        struct MPU9250 *DataStructure) {

	float Acce_X_offset = 0, Acce_Y_offset = 0, Acce_Z_offset = 0;

	for (int i = 0; i < 1000; ++i) {

		MPU9250_Read_Accelerometer(I2Cx, DataStructure);

		Acce_X_offset = Acce_X_offset + DataStructure->Accelerometer_X;
		Acce_Y_offset = Acce_Y_offset + DataStructure->Accelerometer_Y;
		Acce_Z_offset = Acce_Z_offset + DataStructure->Accelerometer_Z;
	}

	DataStructure->Accelerometer_X_offset = Acce_X_offset / 1000;
	DataStructure->Accelerometer_Y_offset = Acce_Y_offset / 1000;
	DataStructure->Accelerometer_Z_offset = Acce_Z_offset / 1000;


	DataStructure->Accelerometer_Z_offset = DataStructure->Accelerometer_Z_offset - DataStructure->Accelerometer_sensitivity_factor;
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Calibration_Gyro(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	        struct MPU9250 *DataStructure) {

	double Gyro_X_offset = 0, Gyro_Y_offset = 0, Gyro_Z_offset = 0;

	for (int i = 0; i < 1000; ++i) {

		MPU9250_Read_Gyroscope(I2Cx, DataStructure);

		Gyro_X_offset += DataStructure->Gyroscope_X;
		Gyro_Y_offset += DataStructure->Gyroscope_Y;
		Gyro_Z_offset += DataStructure->Gyroscope_Z;
	}

	DataStructure->Gyroscope_X_offset = Gyro_X_offset / 1000;
	DataStructure->Gyroscope_Y_offset = Gyro_Y_offset / 1000;
	DataStructure->Gyroscope_Z_offset = Gyro_Z_offset / 1000;

}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
void MPU9250_Calibration_Mag(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	        struct MPU9250 *DataStructure) {

	float X_max = -99999, X_min = 99999, Y_max = -99999, Y_min = 99999, Z_max = -99999, Z_min = 99999;

	/* Hard Iron effect compensation */
	for (int i = 0; i < 1000; ++i) {

		MPU9250_Read_Magnetometer(I2Cx, DataStructure);

		if (DataStructure->Magnetometer_X > X_max)
			X_max = DataStructure->Magnetometer_X;
		if (DataStructure->Magnetometer_Y > Y_max)
			Y_max = DataStructure->Magnetometer_Y;
		if (DataStructure->Magnetometer_Z > Z_max)
			Z_max = DataStructure->Magnetometer_Z;

		if (DataStructure->Magnetometer_X < X_min)
			X_min = DataStructure->Magnetometer_X;
		if (DataStructure->Magnetometer_Y < Y_min)
			Y_min = DataStructure->Magnetometer_Y;
		if (DataStructure->Magnetometer_Z < Z_min)
			Z_min = DataStructure->Magnetometer_Z;

		HAL_Delay(20);
	}

	DataStructure->Magnetometer_X_offset = (X_max + X_min) / 2;
	DataStructure->Magnetometer_Y_offset = (Y_max + Y_min) / 2;
	DataStructure->Magnetometer_Z_offset = (Z_max + Z_min) / 2;

	/* Soft Iron effect compensation */
	float delta_x = (X_max - X_min) / 2;
	float delta_y = (Y_max - Y_min) / 2;
	float delta_z = (Z_max - Z_min) / 2;

	float delta = (delta_x + delta_y + delta_z) / 3;

	DataStructure->Magnetometer_X_scale = delta / delta_x;
	DataStructure->Magnetometer_Y_scale = delta / delta_y;
	DataStructure->Magnetometer_Z_scale = delta / delta_z;
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Set_Offsets(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	   struct MPU9250 *DataStructure,
									   float Acce_X_offset, float Acce_Y_offset, float Acce_Z_offset,
									   float Gyro_X_offset, float Gyro_Y_offset, float Gyro_Z_offset,
									   float Mag_X_offset, float Mag_Y_offset, float Mag_Z_offset,
									   float Mag_X_scale, float Mag_Y_scale, float Mag_Z_scale) {

	DataStructure->Accelerometer_X_offset = Acce_X_offset;
	DataStructure->Accelerometer_Y_offset = Acce_Y_offset;
	DataStructure->Accelerometer_Z_offset = Acce_Z_offset;

	DataStructure->Gyroscope_X_offset = Gyro_X_offset;
	DataStructure->Gyroscope_Y_offset = Gyro_Y_offset;
	DataStructure->Gyroscope_Z_offset = Gyro_Z_offset;

	DataStructure->Magnetometer_X_offset = Mag_X_offset;
	DataStructure->Magnetometer_Y_offset = Mag_Y_offset;
	DataStructure->Magnetometer_Z_offset = Mag_Z_offset;

	DataStructure->Magnetometer_X_scale = Mag_X_scale;
	DataStructure->Magnetometer_Y_scale = Mag_Y_scale;
	DataStructure->Magnetometer_Z_scale = Mag_Z_scale;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

void MPU9250_Calculate_RPY(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  		 struct MPU9250 *DataStructure,
										 double dt) {

	/* Case 1: Read data from AHRS */
	MPU9250_Read_Accelerometer(I2Cx, DataStructure);
	MPU9250_Read_Gyroscope(I2Cx, DataStructure);
	MPU9250_Read_Magnetometer(I2Cx, DataStructure);

	/* Case 2: Calculate accelerometer quaternion */
	double norm = sqrt( pow(DataStructure->Accelerometer_X_g,2) + pow(DataStructure->Accelerometer_Y_g,2) + pow(DataStructure->Accelerometer_Z_g,2) );
	DataStructure->Accelerometer_X_g = DataStructure->Accelerometer_X_g / norm;
	DataStructure->Accelerometer_Y_g = DataStructure->Accelerometer_Y_g / norm;
	DataStructure->Accelerometer_Z_g = DataStructure->Accelerometer_Z_g / norm;

	if( DataStructure->Accelerometer_Z_g >= 0 ) {

		double a = sqrt( 2 * ( DataStructure->Accelerometer_Z_g + 1 ) );

		DataStructure->Accelerometer_quaternion.w = sqrt( (DataStructure->Accelerometer_Z_g + 1) / 2 );
		DataStructure->Accelerometer_quaternion.x = DataStructure->Accelerometer_Y_g / a;
		DataStructure->Accelerometer_quaternion.y = -DataStructure->Accelerometer_X_g / a;
		DataStructure->Accelerometer_quaternion.z = 0;
	}
	else {

		double a = sqrt( 2 * (1 - DataStructure->Accelerometer_Z_g) );
		DataStructure->Accelerometer_quaternion.z = -DataStructure->Accelerometer_Y_g / a;
		DataStructure->Accelerometer_quaternion.y = -sqrt( (1 - DataStructure->Accelerometer_Z_g) / 2 );
		DataStructure->Accelerometer_quaternion.x = 0;
		DataStructure->Accelerometer_quaternion.w = DataStructure->Accelerometer_X_g / a;
	}

	/* Case 3: Calculate gyroscope quaternion */
	struct quaternion temp_quaternion;
	temp_quaternion.w = 0.5 * DataStructure->Gyroscope_quaternion.w;
	temp_quaternion.x = 0.5 * DataStructure->Gyroscope_quaternion.x;
	temp_quaternion.y = 0.5 * DataStructure->Gyroscope_quaternion.y;
	temp_quaternion.z = 0.5 * DataStructure->Gyroscope_quaternion.z;

	struct quaternion gyroscope_vector;
	gyroscope_vector.w = 0;
	gyroscope_vector.x = DataStructure->Gyroscope_X_dgs * (M_PI / 180);
	gyroscope_vector.y = DataStructure->Gyroscope_Y_dgs * (M_PI / 180);
	gyroscope_vector.z = DataStructure->Gyroscope_Z_dgs * (M_PI / 180);

	DataStructure->Gyroscope_quaternion_dot = quaternion_tensor_product(&temp_quaternion, &gyroscope_vector);

	DataStructure->Gyroscope_quaternion.w = DataStructure->Gyroscope_quaternion.w + (DataStructure->Gyroscope_quaternion_dot.w * dt);
	DataStructure->Gyroscope_quaternion.x = DataStructure->Gyroscope_quaternion.x + (DataStructure->Gyroscope_quaternion_dot.x * dt);
	DataStructure->Gyroscope_quaternion.y = DataStructure->Gyroscope_quaternion.y + (DataStructure->Gyroscope_quaternion_dot.y * dt);
	DataStructure->Gyroscope_quaternion.z = DataStructure->Gyroscope_quaternion.z + (DataStructure->Gyroscope_quaternion_dot.z * dt);

	quaternion_normalise(&DataStructure->Gyroscope_quaternion);

	struct rot_matrix gyroscope_matrix;
	quaternion_to_matrix(&DataStructure->Gyroscope_quaternion, &gyroscope_matrix);

	matrix_to_euler(&gyroscope_matrix, &DataStructure->Gyroscope_euler);

	/* Case 4: Calculate magnetometer quaternion */
	double L = pow(DataStructure->Magnetometer_X_uT,2) + pow(DataStructure->Magnetometer_Y_uT,2);

	if( DataStructure->Magnetometer_Y_uT >= 0 ) {

		DataStructure->Magnetometer_quaternion.w = -( sqrt( L + (DataStructure->Magnetometer_X_uT * sqrt(L)) ) ) / ( sqrt(2 * L) );
		DataStructure->Magnetometer_quaternion.x = 0;
		DataStructure->Magnetometer_quaternion.y = 0;
		DataStructure->Magnetometer_quaternion.z = ( DataStructure->Magnetometer_Y_uT ) / ( sqrt(2) * sqrt( L + (DataStructure->Magnetometer_X_uT * sqrt(L)) ) );
	}
	else {

		DataStructure->Magnetometer_quaternion.w = -( DataStructure->Magnetometer_Y_uT ) / ( sqrt(2) * sqrt( L - (DataStructure->Magnetometer_X_uT * sqrt(L)) ) );
		DataStructure->Magnetometer_quaternion.x = 0;
		DataStructure->Magnetometer_quaternion.y = 0;
		DataStructure->Magnetometer_quaternion.z = ( sqrt( L - (DataStructure->Magnetometer_X_uT * sqrt(L)) ) ) / ( sqrt(2 * L) );
	}

	DataStructure->Magnetometer_quaternion = quaternion_tensor_product(&DataStructure->Accelerometer_quaternion, &DataStructure->Magnetometer_quaternion);
	quaternion_normalise(&DataStructure->Magnetometer_quaternion);

	/* Case 5: Calculate accelerometer velocity */
	DataStructure->Accelerometer_X_velocity = DataStructure->Accelerometer_X_velocity_past + DataStructure->Accelerometer_X_g_past + ( (DataStructure->Accelerometer_X_g - DataStructure->Accelerometer_X_g_past) / 2);
	DataStructure->Accelerometer_Y_velocity = DataStructure->Accelerometer_Y_velocity_past + DataStructure->Accelerometer_Y_g_past + ( (DataStructure->Accelerometer_Y_g - DataStructure->Accelerometer_Y_g_past) / 2);
	DataStructure->Accelerometer_Z_velocity = DataStructure->Accelerometer_Z_velocity_past + DataStructure->Accelerometer_Z_g_past + ( (DataStructure->Accelerometer_Z_g - DataStructure->Accelerometer_Z_g_past) / 2);

	/* Case 5: Calculate accelerometer position */
	DataStructure->Accelerometer_X_position = DataStructure->Accelerometer_X_position_past + DataStructure->Accelerometer_X_velocity_past + ( (DataStructure->Accelerometer_X_velocity - DataStructure->Accelerometer_X_velocity_past) / 2);
	DataStructure->Accelerometer_Y_position = DataStructure->Accelerometer_Y_position_past + DataStructure->Accelerometer_Y_velocity_past + ( (DataStructure->Accelerometer_Y_velocity - DataStructure->Accelerometer_Y_velocity_past) / 2);
	DataStructure->Accelerometer_Z_position = DataStructure->Accelerometer_Z_position_past + DataStructure->Accelerometer_Z_velocity_past + ( (DataStructure->Accelerometer_Z_velocity - DataStructure->Accelerometer__velocity_past) / 2);



	DataStructure->Accelerometer_X_g_past = DataStructure->Accelerometer_X_g;
	DataStructure->Accelerometer_Y_g_past = DataStructure->Accelerometer_Y_g;
	DataStructure->Accelerometer_Z_g_past = DataStructure->Accelerometer_Z_g;

	DataStructure->Accelerometer_X_velocity_past = DataStructure->Accelerometer_X_velocity;
	DataStructure->Accelerometer_Y_velocity_past = DataStructure->Accelerometer_Y_velocity;
	DataStructure->Accelerometer_Z_velocity_past = DataStructure->Accelerometer_Z_velocity;

	DataStructure->Accelerometer_X_position_past = DataStructure->Accelerometer_X_position;
	DataStructure->Accelerometer_Y_position_past = DataStructure->Accelerometer_Y_position;
	DataStructure->Accelerometer_Z_position_past = DataStructure->Accelerometer_Z_position;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */
void Complementary_filter(struct MPU9250 *DataStructure,
						  float weight_Roll_Pitch,
						  float weight_Yaw,
						  float dt) {


	if( (DataStructure->Gyroscope_quaternion.w > 0 && DataStructure->Accelerometer_quaternion.w < 0) || (DataStructure->Gyroscope_quaternion.w < 0 && DataStructure->Accelerometer_quaternion.w > 0) ) {

		DataStructure->Accelerometer_quaternion.w = -DataStructure->Accelerometer_quaternion.w;
	}
	if( (DataStructure->Gyroscope_quaternion.x > 0 && DataStructure->Accelerometer_quaternion.x < 0) || (DataStructure->Gyroscope_quaternion.x < 0 && DataStructure->Accelerometer_quaternion.x > 0) ) {

		DataStructure->Accelerometer_quaternion.x = -DataStructure->Accelerometer_quaternion.x;
	}
	if( (DataStructure->Gyroscope_quaternion.y > 0 && DataStructure->Accelerometer_quaternion.y < 0) || (DataStructure->Gyroscope_quaternion.y < 0 && DataStructure->Accelerometer_quaternion.y > 0) ) {

		DataStructure->Accelerometer_quaternion.y = -DataStructure->Accelerometer_quaternion.y;
	}
	if( (DataStructure->Gyroscope_quaternion.z > 0 && DataStructure->Accelerometer_quaternion.z < 0) || (DataStructure->Gyroscope_quaternion.z < 0 && DataStructure->Accelerometer_quaternion.z > 0) ) {

		DataStructure->Accelerometer_quaternion.z = -DataStructure->Accelerometer_quaternion.z;
	}

	DataStructure->Complementary_quaternion.w  = ( (1-weight_Roll_Pitch) * (DataStructure->Complementary_quaternion.w + DataStructure->Gyroscope_quaternion_dot.w * dt )
											   + (weight_Roll_Pitch * DataStructure->Accelerometer_quaternion.w)  );

	DataStructure->Complementary_quaternion.x  = ( (1-weight_Roll_Pitch) * (DataStructure->Complementary_quaternion.x + DataStructure->Gyroscope_quaternion_dot.x * dt )
											   + (weight_Roll_Pitch * DataStructure->Accelerometer_quaternion.x)  );

	DataStructure->Complementary_quaternion.y  = ( (1-weight_Roll_Pitch) * (DataStructure->Complementary_quaternion.y + DataStructure->Gyroscope_quaternion_dot.y * dt )
											   + (weight_Roll_Pitch * DataStructure->Accelerometer_quaternion.y)  );

	DataStructure->Complementary_quaternion.z  = ( (1-weight_Roll_Pitch) * (DataStructure->Complementary_quaternion.z + DataStructure->Gyroscope_quaternion_dot.z * dt )
											   + (weight_Roll_Pitch * DataStructure->Accelerometer_quaternion.z)  );

	quaternion_normalise(&DataStructure->Complementary_quaternion);
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

void Kalman_filter(struct MPU9250 *DataStructure,
				   float Q_Roll_Pitch, float R_Roll_Pitch,
				   float Q_Yaw, float R_Yaw,
				   float dt) {

	/* Case 1: Update Q and R value */
	/*
	if( DataStructure->Kalman_P.kalman_Q != Q_Roll_Pitch || DataStructure->Kalman_P.kalman_R != R_Roll_Pitch ) {

		Kalman_filter_init(&DataStructure->Kalman_R, Q_Roll_Pitch, R_Roll_Pitch);
		Kalman_filter_init(&DataStructure->Kalman_P, Q_Roll_Pitch, R_Roll_Pitch);
		Kalman_filter_init(&DataStructure->Kalman_Y, Q_Yaw, R_Yaw);

		return;
	}
	*/

	/* Case 2: */
	//DataStructure->Kalman_filter_Roll  = Kalman_filter_calculate(&DataStructure->Kalman_R, DataStructure->Accelerometer_Roll,  DataStructure->Gyroscope_X_dgs, dt);
	//DataStructure->Kalman_filter_Pitch = Kalman_filter_calculate(&DataStructure->Kalman_P, DataStructure->Accelerometer_Pitch, DataStructure->Gyroscope_Y_dgs, dt);
	//DataStructure->Kalman_filter_Yaw   = Kalman_filter_calculate(&DataStructure->Kalman_Y, DataStructure->Magnetometer_Yaw,    DataStructure->Gyroscope_Z_dgs, dt);
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

void Madgwick_filter(struct MPU9250 *DataStructure,
					 float beta,
					 float dt) {

	MadgwickAHRSupdate(beta,
					   DataStructure->Gyroscope_X_dgs * (M_PI / 180), DataStructure->Gyroscope_Y_dgs * (M_PI / 180), DataStructure->Gyroscope_Z_dgs * (M_PI / 180),
					   DataStructure->Accelerometer_X_g, DataStructure->Accelerometer_Y_g, DataStructure->Accelerometer_Z_g,
					   DataStructure->Magnetometer_X_uT, DataStructure->Magnetometer_Y_uT, DataStructure->Magnetometer_Z_uT,
					   dt);

	DataStructure->Madgwick_quaternion.w = q0;
	DataStructure->Madgwick_quaternion.x = q1;
	DataStructure->Madgwick_quaternion.y = q2;
	DataStructure->Madgwick_quaternion.z = q3;

	quaternion_normalise(&DataStructure->Madgwick_quaternion);

	struct rot_matrix madgwick_matrix;
	quaternion_to_matrix(&DataStructure->Madgwick_quaternion, &madgwick_matrix);

	matrix_to_euler(&madgwick_matrix, &DataStructure->Madgwick_euler);
}
/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void Mahony_filter(struct MPU9250 *DataStructure, float dt) {

	MahonyAHRSupdate(DataStructure->Gyroscope_X_dgs * (M_PI / 180), DataStructure->Gyroscope_Y_dgs * (M_PI / 180), DataStructure->Gyroscope_Z_dgs * (M_PI / 180),
			   	   	 DataStructure->Accelerometer_X_g, DataStructure->Accelerometer_Y_g, DataStructure->Accelerometer_Z_g,
					 DataStructure->Magnetometer_X_uT, DataStructure->Magnetometer_Y_uT, DataStructure->Magnetometer_Z_uT,
					 dt);

	float q0 = Mahony_q0;
	float q1 = Mahony_q1;
	float q2 = Mahony_q2;
	float q3 = Mahony_q3;

	DataStructure->Mahony_filter_Roll  = atan2f( 2 * (q0*q1 + q2*q3), 1 - 2 * (powf(q1,2) + powf(q2,2)) ) * (180 / M_PI);
	DataStructure->Mahony_filter_Pitch = asinf( 2 * (q0*q2 - q3*q1) ) 									  * (180 / M_PI);
	DataStructure->Mahony_filter_Yaw   = atan2f( 2 * (q0*q3 + q1*q2), 1 - 2 * (powf(q2,2) + powf(q3,2)) ) * (180 / M_PI);
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */
