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

	/* Case 2: Save configuration to data structure */
	if(      Range == MPU9250_Acce_2G )     DataStructure->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_2G;
	else if( Range == MPU9250_Acce_4G )		DataStructure->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_4G;
	else if( Range == MPU9250_Acce_8G )		DataStructure->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_8G;
	else if( Range == MPU9250_Acce_16G )	DataStructure->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_16G;


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

	/* Case 2: Save configuration to data structure */
	if(      Range == MPU9250_Gyro_250s )   DataStructure->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_250s;
	else if( Range == MPU9250_Gyro_500s )	DataStructure->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_500s;
	else if( Range == MPU9250_Gyro_1000s )	DataStructure->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_1000s;
	else if( Range == MPU9250_Gyro_2000s )	DataStructure->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_2000s;


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

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Init_FAIL;
	}

	/* Case 2: Who am i test */
	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_WHO_AM_I, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Init_FAIL;
	}

	if( Byte_temp != 0x71 ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Init_FAIL;
	}

	/* Case 3: Wake up */
	Byte_temp = 0x01;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Device_addres, MPU9250_PWR_MGMT_1, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Init_FAIL;
	}

	/* Case 4: Accelerometer configuration */
	if( MPU9250_Accelerometer_Configuration(I2Cx, DataStructure, Acce_range) != MPU9250_Accelerometer_Config_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Accelerometer_Config_FAIL;
	}

	/* Case 5: Gyroscope configuration */
	if( MPU9250_Gyroscope_Configuration(I2Cx, DataStructure, Gyro_range) != MPU9250_Gyroscope_Config_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Gyroscope_Config_FAIL;
	}

	/* Case 6: Magnetometer configuration */
	if( MPU9250_Magnetometer_Configuration(I2Cx, DataStructure) != MPU9250_Magnetometer_Config_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Magnetometer_Config_FAIL;
	}

	return MPU9250_Init_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Accelerometer(I2C_HandleTypeDef *I2Cx,
											  struct MPU9250 *DataStructure) {

	uint8_t Bytes_temp[6] = {0x00};

	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_ACCEL_XOUT_H, 1, Bytes_temp , 6, 1000) != HAL_OK ) {

		return MPU9250_Read_Accelerometer_FAIL;
	}

	DataStructure->Accelerometer_X = Bytes_temp[0] << 8 | Bytes_temp[1];
	DataStructure->Accelerometer_Y = Bytes_temp[2] << 8 | Bytes_temp[3];
	DataStructure->Accelerometer_Z = Bytes_temp[4] << 8 | Bytes_temp[5];

	/* Case x: Calculate g-force values for XYZ axis */
	DataStructure->Accelerometer_X_g = (float)(DataStructure->Accelerometer_X) / DataStructure->Accelerometer_sensitivity_factor;
	DataStructure->Accelerometer_Y_g = (float)(DataStructure->Accelerometer_Y) / DataStructure->Accelerometer_sensitivity_factor;
	DataStructure->Accelerometer_Z_g = (float)(DataStructure->Accelerometer_Z) / DataStructure->Accelerometer_sensitivity_factor;

	/* Case x: Calculate Roll, Pitch, Yaw values */
	DataStructure->Accelerometer_Roll  = atan2(DataStructure->Accelerometer_Y_g, DataStructure->Accelerometer_Z_g) * (180 / M_PI);
	DataStructure->Accelerometer_Pitch = atan2(DataStructure->Accelerometer_X_g, DataStructure->Accelerometer_Z_g) * (180 / M_PI);
	DataStructure->Accelerometer_Yaw = 0;

	return MPU9250_Read_Accelerometer_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Gyroscope(I2C_HandleTypeDef *I2Cx,
										  struct MPU9250 *DataStructure,
										  float dt) {

	uint8_t Bytes_temp[6] = { 0x00 };

	if (HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_GYRO_XOUT_H, 1, Bytes_temp, 6, 1000) != HAL_OK) {

		return MPU9250_Read_Gyroscope_FAIL;
	}

	DataStructure->Gyroscope_X = Bytes_temp[0] << 8 | Bytes_temp[1];
	DataStructure->Gyroscope_Y = Bytes_temp[2] << 8 | Bytes_temp[3];
	DataStructure->Gyroscope_Z = Bytes_temp[4] << 8 | Bytes_temp[5];

	/* Case x: Calculate dgs/s values for XYZ axis */
	DataStructure->Gyroscope_X_dgs = (float)(DataStructure->Gyroscope_X) / DataStructure->Gyroscope_sensitivity_factor;
	DataStructure->Gyroscope_Y_dgs = (float)(DataStructure->Gyroscope_Y) / DataStructure->Gyroscope_sensitivity_factor;
	DataStructure->Gyroscope_Z_dgs = (float)(DataStructure->Gyroscope_Z) / DataStructure->Gyroscope_sensitivity_factor;

	/* Case x: Calculate Roll, Pitch, Yaw values */
	DataStructure->Gyroscope_Roll  = DataStructure->Gyroscope_Roll  + ( 0.5 * ( dt )
													* (DataStructure->Gyroscope_X_dgs + DataStructure->Gyroscope_X_dgs_past) );

	DataStructure->Gyroscope_Pitch = DataStructure->Gyroscope_Pitch + ( 0.5 * ( dt )
													* (DataStructure->Gyroscope_Y_dgs + DataStructure->Gyroscope_Y_dgs_past) );

	/* Case x: Save actual dgs/s value to data structure */
	DataStructure->Gyroscope_X_dgs_past = DataStructure->Gyroscope_X_dgs;
	DataStructure->Gyroscope_Y_dgs_past = DataStructure->Gyroscope_Y_dgs;
	DataStructure->Gyroscope_Z_dgs_past = DataStructure->Gyroscope_Z_dgs;

	return MPU9250_Read_Gyroscope_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Magnetometer(I2C_HandleTypeDef *I2Cx,
										     struct MPU9250 *DataStructure) {

	uint8_t Bytes_temp[8] = { 0x00 };

	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Magnetometer_addres, AK9863_ST1, 1, Bytes_temp, 8, 1000) != HAL_OK ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Read_Magnetometer_FAIL;
	}

	if( Bytes_temp[0] & 0x00 ) {

		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		return MPU9250_Read_Magnetometer_FAIL;
	}

	DataStructure->Magnetometer_X = Bytes_temp[1] << 8 | Bytes_temp[2];
	DataStructure->Magnetometer_Y = Bytes_temp[3] << 8 | Bytes_temp[4];
	DataStructure->Magnetometer_Z = Bytes_temp[5] << 8 | Bytes_temp[6];

	/* Case x: Calculate uT (micro Tesla) value for XYZ axis */
	DataStructure->Magnetometer_X_uT = DataStructure->Magnetometer_X * DataStructure->Magnetometer_ASAX * DataStructure->Magnetometer_sesitivity_factor;
	DataStructure->Magnetometer_Y_uT = DataStructure->Magnetometer_Y * DataStructure->Magnetometer_ASAY * DataStructure->Magnetometer_sesitivity_factor;
	DataStructure->Magnetometer_Z_uT = DataStructure->Magnetometer_Z * DataStructure->Magnetometer_ASAZ * DataStructure->Magnetometer_sesitivity_factor;

	return MPU9250_Read_Magnetometer_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */
