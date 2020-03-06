/*
 * mpu9250.c
 *
 *  Created on: 04.07.2019
 *  Author: mnowak5
 */

#include "mpu9250.h"

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Accelerometer_Configuration(I2C_HandleTypeDef *I2Cx,
													   struct MPU9250 *mpu,
													   MPU9250_Acce_range Range) {

	uint8_t Byte_temp = 0x00;

	/* Case 1: Set accelerometer sensitivity range */
	Byte_temp = Range << 3;

	if( HAL_I2C_Mem_Write(I2Cx, mpu->Device_addres, MPU9250_ACCEL_CONFIG, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Accelerometer_Config_FAIL;
	}

	/* Case 2: Set accelerometer low pass filter cut-off frequency */
	/*
	Byte_temp = 0x0E;

	if( HAL_I2C_Mem_Write(I2Cx, mpu->Device_addres, MPU9250_ACCEL_CONFIG_2, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Accelerometer_Config_FAIL;
	}
	*/

	/* Case 3: Save configuration to data structure */
	if(      Range == MPU9250_Acce_2G )     mpu->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_2G;
	else if( Range == MPU9250_Acce_4G )		mpu->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_4G;
	else if( Range == MPU9250_Acce_8G )		mpu->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_8G;
	else if( Range == MPU9250_Acce_16G )	mpu->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_16G;

	mpu->Accelerometer_X_offset = 0;
	mpu->Accelerometer_Y_offset = 0;
	mpu->Accelerometer_Z_offset = 0;

	return MPU9250_Accelerometer_Config_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Gyroscope_Configuration(I2C_HandleTypeDef *I2Cx,
												   struct MPU9250 *mpu,
												   MPU9250_Gyro_range Range) {

	uint8_t Byte_temp = 0x00;


	/* Case 1: Set gyroscope sensitivity range */
	Byte_temp = Range << 3;

	if( HAL_I2C_Mem_Write(I2Cx, mpu->Device_addres, MPU9250_GYRO_CONFIG, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Gyroscope_Config_FAIL;
	}

	/* Case 2: Set gyroscope low pass filter cut-off frequency */
	Byte_temp = 0x0E;

	if( HAL_I2C_Mem_Write(I2Cx, mpu->Device_addres, MPU9250_CONFIG, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Gyroscope_Config_FAIL;
	}

	/* Case 3: Save configuration to data structure */
	if(      Range == MPU9250_Gyro_250s )   mpu->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_250s;
	else if( Range == MPU9250_Gyro_500s )	mpu->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_500s;
	else if( Range == MPU9250_Gyro_1000s )	mpu->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_1000s;
	else if( Range == MPU9250_Gyro_2000s )	mpu->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_2000s;

	mpu->Gyroscope_X_offset = 0;
	mpu->Gyroscope_Y_offset = 0;
	mpu->Gyroscope_Z_offset = 0;

	return MPU9250_Gyroscope_Config_OK;

}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Magnetometer_Configuration(I2C_HandleTypeDef *I2Cx,
												      struct MPU9250 *mpu) {


	uint8_t Byte_temp = 0x00;
	uint8_t Bytes_temp[3] = {0};

	mpu->Magnetometer_addres = 0x0C << 1;
	mpu->Magnetometer_sesitivity_factor = 0.1499; /* 4912/32768 */

	// Case 2: Disable the I2C master interface
	Byte_temp = 0x00;

	if( HAL_I2C_Mem_Write(I2Cx, mpu->Device_addres, MPU9250_USER_CTRL, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	// Case 3: Enable the bypass multiplexer
	Byte_temp = 0x02;

	if( HAL_I2C_Mem_Write(I2Cx, mpu->Device_addres, MPU9250_INT_PIN_CFG, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	// Case 1: Is device connected ?
	if( HAL_I2C_IsDeviceReady(I2Cx, mpu->Magnetometer_addres, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	// Case 2: Who am i test
	if( HAL_I2C_Mem_Read(I2Cx, mpu->Magnetometer_addres, AK9863_WIA, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Init_FAIL;
	}

	if( Byte_temp != 0x48 ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Init_FAIL;
	}

	// Case 4: Setup to fuse ROM access mode and 16-bit output
	Byte_temp = 0x1F;

	if( HAL_I2C_Mem_Write(I2Cx, mpu->Magnetometer_addres, AK9863_CNTL1, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	HAL_Delay(100);

	// Case 5: Read from the fuse ROM sensitivity adjustment values
	if( HAL_I2C_Mem_Read(I2Cx, mpu->Magnetometer_addres, AK9863_ASAX | 0x80, 1, Bytes_temp, 3, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	mpu->Magnetometer_ASAX = ( ( (Bytes_temp[0] - 128) * 0.5 ) / 128 ) + 1;
	mpu->Magnetometer_ASAY = ( ( (Bytes_temp[1] - 128) * 0.5 ) / 128 ) + 1;
	mpu->Magnetometer_ASAZ = ( ( (Bytes_temp[2] - 128) * 0.5 ) / 128 ) + 1;

	// Case 6: Reset to power down mode
	Byte_temp = 0x00;

	if( HAL_I2C_Mem_Write(I2Cx, mpu->Magnetometer_addres, AK9863_CNTL1, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	// Case 7: Enable continuous mode 2 and 16-bit output
	Byte_temp = 0x16; // 0x16

	if( HAL_I2C_Mem_Write(I2Cx, mpu->Magnetometer_addres, AK9863_CNTL1, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	HAL_Delay(100);

	return MPU9250_Magnetometer_Config_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Init(I2C_HandleTypeDef *I2Cx,
								struct MPU9250 *mpu,
								MPU9250_Device_number Number,
								MPU9250_Acce_range Acce_range,
								MPU9250_Gyro_range Gyro_range) {

	uint8_t Byte_temp = 0x00;

	mpu->Device_number = Number;
	mpu->Device_addres = (0x68 | mpu->Device_number) << 1;

	/* Case 1: Is device connected ? */
	if( HAL_I2C_IsDeviceReady(I2Cx, mpu->Device_addres, 1, 1000) != HAL_OK ) {

		return MPU9250_Init_FAIL;
	}

	/* Case 2: Who am i test */
	if( HAL_I2C_Mem_Read(I2Cx, mpu->Device_addres, MPU9250_WHO_AM_I, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Init_FAIL;
	}

	if( Byte_temp != 0x71 ) {

		return MPU9250_Init_FAIL;
	}

	/* Case 3: Wake up */
	Byte_temp = 0x01;

	if( HAL_I2C_Mem_Write(I2Cx, mpu->Device_addres, MPU9250_PWR_MGMT_1, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Init_FAIL;
	}

	/* Case 4: Accelerometer configuration */
	if( MPU9250_Accelerometer_Configuration(I2Cx, mpu, Acce_range) != MPU9250_Accelerometer_Config_OK ) {

		return MPU9250_Accelerometer_Config_FAIL;
	}

	/* Case 5: Gyroscope configuration */
	if( MPU9250_Gyroscope_Configuration(I2Cx, mpu, Gyro_range) != MPU9250_Gyroscope_Config_OK ) {

		return MPU9250_Gyroscope_Config_FAIL;
	}

	/* Case 6: Magnetometer configuration */
	if( MPU9250_Magnetometer_Configuration(I2Cx, mpu) != MPU9250_Magnetometer_Config_OK ) {

		return MPU9250_Magnetometer_Config_FAIL;
	}

	/* Case 7: Default variables value */
	mpu->Magnetometer_X_scale = 1;
	mpu->Magnetometer_Y_scale = 1;
	mpu->Magnetometer_Z_scale = 1;

	mpu->Magnetometer_X_offset = 0;
	mpu->Magnetometer_Y_offset = 0;
	mpu->Magnetometer_Z_offset = 0;

	mpu->Accelerometer_vector_velocity.x = 0;
	mpu->Accelerometer_vector_velocity.y = 0;
	mpu->Accelerometer_vector_velocity.z = 0;

	mpu->Accelerometer_vector_position.x = 0;
	mpu->Accelerometer_vector_position.y = 0;
	mpu->Accelerometer_vector_position.z = 0;

	return MPU9250_Init_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Accelerometer(I2C_HandleTypeDef *I2Cx,
											  struct MPU9250 *mpu) {

	uint8_t Bytes_temp[6] = {0x00};

	if( HAL_I2C_Mem_Read(I2Cx, mpu->Device_addres, MPU9250_ACCEL_XOUT_H, 1, Bytes_temp , 6, 1000) != HAL_OK ) {

		return MPU9250_Read_Accelerometer_FAIL;
	}

	mpu->Accelerometer_X = ( Bytes_temp[0] << 8 | Bytes_temp[1] ) - mpu->Accelerometer_X_offset;
	mpu->Accelerometer_Y = ( Bytes_temp[2] << 8 | Bytes_temp[3] ) - mpu->Accelerometer_Y_offset;
	mpu->Accelerometer_Z = ( Bytes_temp[4] << 8 | Bytes_temp[5] ) - mpu->Accelerometer_Z_offset;

	/* Case x: Calculate g-force values for XYZ axis */
	mpu->Accelerometer_vector_g.x = (float)(mpu->Accelerometer_X) / mpu->Accelerometer_sensitivity_factor;
	mpu->Accelerometer_vector_g.y = (float)(mpu->Accelerometer_Y) / mpu->Accelerometer_sensitivity_factor;
	mpu->Accelerometer_vector_g.z = (float)(mpu->Accelerometer_Z) / mpu->Accelerometer_sensitivity_factor;

	return MPU9250_Read_Accelerometer_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Gyroscope(I2C_HandleTypeDef *I2Cx,
										  struct MPU9250 *mpu) {

	uint8_t Bytes_temp[6] = { 0x00 };

	if (HAL_I2C_Mem_Read(I2Cx, mpu->Device_addres, MPU9250_GYRO_XOUT_H, 1, Bytes_temp, 6, 1000) != HAL_OK) {

		return MPU9250_Read_Gyroscope_FAIL;
	}

	mpu->Gyroscope_X = ( Bytes_temp[0] << 8 | Bytes_temp[1] ) - mpu->Gyroscope_X_offset;
	mpu->Gyroscope_Y = ( Bytes_temp[2] << 8 | Bytes_temp[3] ) - mpu->Gyroscope_Y_offset;
	mpu->Gyroscope_Z = ( Bytes_temp[4] << 8 | Bytes_temp[5] ) - mpu->Gyroscope_Z_offset;

	/* Case x: Calculate dgs/s values for XYZ axis */
	mpu->Gyroscope_X_dgs =  (float)(mpu->Gyroscope_X) / mpu->Gyroscope_sensitivity_factor;
	mpu->Gyroscope_Y_dgs =  (float)(mpu->Gyroscope_Y) / mpu->Gyroscope_sensitivity_factor;
	mpu->Gyroscope_Z_dgs =  (float)(mpu->Gyroscope_Z) / mpu->Gyroscope_sensitivity_factor;

	return MPU9250_Read_Gyroscope_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Magnetometer(I2C_HandleTypeDef *I2Cx,
										     struct MPU9250 *mpu) {

	uint8_t Bytes_temp[8] = { 0x00 };

	/* Case x: Read measured values from registers */
	if( HAL_I2C_Mem_Read(I2Cx, mpu->Magnetometer_addres, AK9863_ST1, 1, Bytes_temp, 8, 1000) != HAL_OK ) {

		return MPU9250_Read_Magnetometer_FAIL;
	}

	if( Bytes_temp[0] & 0x00 ) {

		return MPU9250_Read_Magnetometer_FAIL;
	}

	mpu->Magnetometer_X = ( ( Bytes_temp[2] << 8 | Bytes_temp[1] ) - mpu->Magnetometer_X_offset );
	mpu->Magnetometer_Y = ( ( Bytes_temp[4] << 8 | Bytes_temp[3] ) - mpu->Magnetometer_Y_offset );
	mpu->Magnetometer_Z = ( ( Bytes_temp[6] << 8 | Bytes_temp[5] ) - mpu->Magnetometer_Z_offset );

	/* Case x: Calculate uT (micro Tesla) value for XYZ axis */
	mpu->Magnetometer_X_uT = mpu->Magnetometer_X * mpu->Magnetometer_ASAX * mpu->Magnetometer_sesitivity_factor * mpu->Magnetometer_X_scale;
	mpu->Magnetometer_Y_uT = mpu->Magnetometer_Y * mpu->Magnetometer_ASAY * mpu->Magnetometer_sesitivity_factor * mpu->Magnetometer_Y_scale;
	mpu->Magnetometer_Z_uT = mpu->Magnetometer_Z * mpu->Magnetometer_ASAZ * mpu->Magnetometer_sesitivity_factor * mpu->Magnetometer_Z_scale;

	float a = mpu->Magnetometer_X_uT;
	float b = mpu->Magnetometer_Y_uT;
	float c = mpu->Magnetometer_Z_uT;

	mpu->Magnetometer_X_uT = b;
	mpu->Magnetometer_Y_uT = a;
	mpu->Magnetometer_Z_uT = -c;

	return MPU9250_Read_Magnetometer_OK;
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Calibration_Acce(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	        struct MPU9250 *mpu) {

	float Acce_X_offset = 0, Acce_Y_offset = 0, Acce_Z_offset = 0;

	for (int i = 0; i < 1000; ++i) {

		MPU9250_Read_Accelerometer(I2Cx, mpu);

		Acce_X_offset = Acce_X_offset + mpu->Accelerometer_X;
		Acce_Y_offset = Acce_Y_offset + mpu->Accelerometer_Y;
		Acce_Z_offset = Acce_Z_offset + mpu->Accelerometer_Z;
	}

	mpu->Accelerometer_X_offset = Acce_X_offset / 1000;
	mpu->Accelerometer_Y_offset = Acce_Y_offset / 1000;
	mpu->Accelerometer_Z_offset = Acce_Z_offset / 1000;


	mpu->Accelerometer_Z_offset = mpu->Accelerometer_Z_offset - mpu->Accelerometer_sensitivity_factor;
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Calibration_Gyro(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	        struct MPU9250 *mpu) {

	float Gyro_X_offset = 0, Gyro_Y_offset = 0, Gyro_Z_offset = 0;

	for (int i = 0; i < 1000; ++i) {

		MPU9250_Read_Gyroscope(I2Cx, mpu);

		Gyro_X_offset += mpu->Gyroscope_X;
		Gyro_Y_offset += mpu->Gyroscope_Y;
		Gyro_Z_offset += mpu->Gyroscope_Z;
	}

	mpu->Gyroscope_X_offset = Gyro_X_offset / 1000;
	mpu->Gyroscope_Y_offset = Gyro_Y_offset / 1000;
	mpu->Gyroscope_Z_offset = Gyro_Z_offset / 1000;

}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
void MPU9250_Calibration_Mag(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	        struct MPU9250 *mpu) {

	float X_max = -99999, X_min = 99999, Y_max = -99999, Y_min = 99999, Z_max = -99999, Z_min = 99999;

	/* Hard Iron effect compensation */
	for (int i = 0; i < 1000; ++i) {

		MPU9250_Read_Magnetometer(I2Cx, mpu);

		if (mpu->Magnetometer_X > X_max)
			X_max = mpu->Magnetometer_X;
		if (mpu->Magnetometer_Y > Y_max)
			Y_max = mpu->Magnetometer_Y;
		if (mpu->Magnetometer_Z > Z_max)
			Z_max = mpu->Magnetometer_Z;

		if (mpu->Magnetometer_X < X_min)
			X_min = mpu->Magnetometer_X;
		if (mpu->Magnetometer_Y < Y_min)
			Y_min = mpu->Magnetometer_Y;
		if (mpu->Magnetometer_Z < Z_min)
			Z_min = mpu->Magnetometer_Z;

		HAL_Delay(20);
	}

	mpu->Magnetometer_X_offset = (X_max + X_min) / 2;
	mpu->Magnetometer_Y_offset = (Y_max + Y_min) / 2;
	mpu->Magnetometer_Z_offset = (Z_max + Z_min) / 2;

	/* Soft Iron effect compensation */
	float delta_x = (X_max - X_min) / 2;
	float delta_y = (Y_max - Y_min) / 2;
	float delta_z = (Z_max - Z_min) / 2;

	float delta = (delta_x + delta_y + delta_z) / 3;

	mpu->Magnetometer_X_scale = delta / delta_x;
	mpu->Magnetometer_Y_scale = delta / delta_y;
	mpu->Magnetometer_Z_scale = delta / delta_z;
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Set_Offsets(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	   struct MPU9250 *mpu,
									   float Acce_X_offset, float Acce_Y_offset, float Acce_Z_offset,
									   float Gyro_X_offset, float Gyro_Y_offset, float Gyro_Z_offset,
									   float Mag_X_offset, float Mag_Y_offset, float Mag_Z_offset,
									   float Mag_X_scale, float Mag_Y_scale, float Mag_Z_scale) {

	mpu->Accelerometer_X_offset = Acce_X_offset;
	mpu->Accelerometer_Y_offset = Acce_Y_offset;
	mpu->Accelerometer_Z_offset = Acce_Z_offset;

	mpu->Gyroscope_X_offset = Gyro_X_offset;
	mpu->Gyroscope_Y_offset = Gyro_Y_offset;
	mpu->Gyroscope_Z_offset = Gyro_Z_offset;

	mpu->Magnetometer_X_offset = Mag_X_offset;
	mpu->Magnetometer_Y_offset = Mag_Y_offset;
	mpu->Magnetometer_Z_offset = Mag_Z_offset;

	mpu->Magnetometer_X_scale = Mag_X_scale;
	mpu->Magnetometer_Y_scale = Mag_Y_scale;
	mpu->Magnetometer_Z_scale = Mag_Z_scale;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

void MPU9250_Update(I2C_HandleTypeDef *I2Cx,
				    struct MPU9250 *mpu,
				    float dt) {

	/* Case 1: Read data from AHRS */
	MPU9250_Read_Accelerometer(I2Cx, mpu);
	MPU9250_Read_Gyroscope(I2Cx, mpu);
	MPU9250_Read_Magnetometer(I2Cx, mpu);

	/* Case 2: Calculate AHRS orientation */
	/*
	MadgwickAHRSupdate(mpu->Gyroscope_X_dgs * DEG_TO_RAD, mpu->Gyroscope_Y_dgs * DEG_TO_RAD, mpu->Gyroscope_Z_dgs * DEG_TO_RAD,
					   mpu->Accelerometer_vector_g.x, mpu->Accelerometer_vector_g.y, mpu->Accelerometer_vector_g.z,
					   mpu->Magnetometer_X_uT, mpu->Magnetometer_Y_uT, mpu->Magnetometer_Z_uT,
					   dt);

	mpu->Madgwick_quaternion.w = q0;
	mpu->Madgwick_quaternion.x = q1;
	mpu->Madgwick_quaternion.y = q2;
	mpu->Madgwick_quaternion.z = q3;

	quaternion_normalise(&mpu->Madgwick_quaternion);

	struct rot_matrix madgwick_matrix;
	quaternion_to_matrix(&mpu->Madgwick_quaternion, &madgwick_matrix);

	matrix_to_euler(&madgwick_matrix, &mpu->Madgwick_euler);
	*/

	/* Case 3: Delete g-force vector */
	mpu->Accelerometer_vector_g_offset.x = 0.9 * mpu->Accelerometer_vector_g_offset.x + (1 - 0.9) * mpu->Accelerometer_vector_g.x;
	mpu->Accelerometer_vector_g_offset.y = 0.9 * mpu->Accelerometer_vector_g_offset.y + (1 - 0.9) * mpu->Accelerometer_vector_g.y;
	mpu->Accelerometer_vector_g_offset.z = 0.9 * mpu->Accelerometer_vector_g_offset.z + (1 - 0.9) * mpu->Accelerometer_vector_g.z;

	mpu->Accelerometer_vector_without_g.x = (mpu->Accelerometer_vector_g.x - mpu->Accelerometer_vector_g_offset.x);
	mpu->Accelerometer_vector_without_g.y = (mpu->Accelerometer_vector_g.y - mpu->Accelerometer_vector_g_offset.y);
	mpu->Accelerometer_vector_without_g.z = (mpu->Accelerometer_vector_g.z - mpu->Accelerometer_vector_g_offset.z);

	/* Case 4: Calculate AHRS velocity */
	if( fabs(mpu->Accelerometer_vector_without_g.x) < 0.05 ) mpu->Accelerometer_vector_velocity.x = 0;
	if( fabs(mpu->Accelerometer_vector_without_g.y) < 0.05 ) mpu->Accelerometer_vector_velocity.y = 0;
	if( fabs(mpu->Accelerometer_vector_without_g.z) < 0.05 ) mpu->Accelerometer_vector_velocity.z = 0;

	mpu->Accelerometer_vector_velocity.x += mpu->Accelerometer_vector_without_g.x * G_TO_MS2 * dt;
	mpu->Accelerometer_vector_velocity.y += mpu->Accelerometer_vector_without_g.y * G_TO_MS2 * dt;
	mpu->Accelerometer_vector_velocity.z += mpu->Accelerometer_vector_without_g.z * G_TO_MS2 * dt;

	/* Case 5: Calculate AHRS positon */
	mpu->Accelerometer_vector_position.x += mpu->Accelerometer_vector_velocity.x * dt;
	mpu->Accelerometer_vector_position.y += mpu->Accelerometer_vector_velocity.y * dt;
	mpu->Accelerometer_vector_position.z += mpu->Accelerometer_vector_velocity.z * dt;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */
