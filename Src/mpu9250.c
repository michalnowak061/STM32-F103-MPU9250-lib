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

	DataStructure->Magnetometer_addres = 0x0C << 1;


	/* Case 1: ? */
	Byte_temp = 0x02;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Device_addres, MPU9250_INT_PIN_CFG, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Magnetometer_Config_FAIL;
	}

	/* Case 2: Is device connected ? */
	if( HAL_I2C_IsDeviceReady(I2Cx, DataStructure->Magnetometer_addres, 1, 1000) != HAL_OK ) {

		return MPU9250_Magnetometer_Config_FAIL;
	}

	/* Case 3: ? */
	Byte_temp = 0x00;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Magnetometer_addres, MPU9250_CNTL, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Magnetometer_Config_FAIL;
	}

	/* Case 3: ? */
	Byte_temp = 0x01;

	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Magnetometer_addres, MPU9250_RSV, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Magnetometer_Config_FAIL;
	}


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
	if( HAL_I2C_Mem_Write(I2Cx, DataStructure->Device_addres, MPU9250_PWR_MGMT_1, 1, 0x00, 1, 1000) != HAL_OK ) {

		return MPU9250_Init_FAIL;
	}

	/* Case 4: Accelerometer configuration */
	if( MPU9250_Accelerometer_Configuration(I2Cx, DataStructure, Acce_range) != MPU9250_Accelerometer_Config_OK ) {

		return MPU9250_Accelerometer_Config_FAIL;
	}

	/* Case 5: Accelerometer configuration */
	if( MPU9250_Gyroscope_Configuration(I2Cx, DataStructure, Gyro_range) != MPU9250_Gyroscope_Config_OK ) {

		return MPU9250_Gyroscope_Config_FAIL;
	}

	/* Case 6: Magnetometer configuration */
	if( MPU9250_Magnetometer_Configuration(I2Cx, DataStructure) != MPU9250_Magnetometer_Config_OK ) {

		return MPU9250_Magnetometer_Config_FAIL;
	}


	return MPU9250_Init_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Accelerometer(I2C_HandleTypeDef *I2Cx,
											  struct MPU9250 *DataStructure) {

	uint8_t Bytes_temp[2] = {0x00};


	/* Case 1: X Axis */
	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_ACCEL_XOUT_H | 0x80, 1, Bytes_temp , 2, 1000) != HAL_OK ) {

		return MPU9250_Read_Accelerometer_FAIL;
	}

	DataStructure->Accelerometer_X = Bytes_temp[0] << 8 | Bytes_temp[1];

	/* Case 2: Y Axis */
	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_ACCEL_YOUT_H | 0x80, 1, Bytes_temp , 2, 1000) != HAL_OK ) {

		return MPU9250_Read_Accelerometer_FAIL;
	}

	DataStructure->Accelerometer_Y = Bytes_temp[0] << 8 | Bytes_temp[1];

	/* Case 3: Z Axis */
	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_ACCEL_ZOUT_H | 0x80, 1, Bytes_temp , 2, 1000) != HAL_OK ) {

		return MPU9250_Read_Accelerometer_FAIL;
	}

	DataStructure->Accelerometer_Z = Bytes_temp[0] << 8 | Bytes_temp[1];


	return MPU9250_Read_Accelerometer_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Gyroscope(I2C_HandleTypeDef *I2Cx,
										  struct MPU9250 *DataStructure) {

	uint8_t Bytes_temp[2] = { 0x00 };

	/* Case 1: X Axis */
	if (HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_GYRO_XOUT_H | 0x80, 1, Bytes_temp, 2, 1000) != HAL_OK) {

		return MPU9250_Read_Gyroscope_FAIL;
	}

	DataStructure->Gyroscope_X = Bytes_temp[0] << 8 | Bytes_temp[1];

	/* Case 2: Y Axis */
	if (HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_GYRO_YOUT_H | 0x80, 1, Bytes_temp, 2, 1000) != HAL_OK) {

		return MPU9250_Read_Gyroscope_FAIL;
	}

	DataStructure->Gyroscope_Y = Bytes_temp[0] << 8 | Bytes_temp[1];

	/* Case 3: Z Axis */
	if (HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_GYRO_ZOUT_H | 0x80, 1, Bytes_temp, 2, 1000) != HAL_OK) {

		return MPU9250_Read_Gyroscope_FAIL;
	}

	DataStructure->Gyroscope_Z = Bytes_temp[0] << 8 | Bytes_temp[1];

	return MPU9250_Read_Gyroscope_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Read_Magnetometer(I2C_HandleTypeDef *I2Cx,
										     struct MPU9250 *DataStructure) {


	uint8_t Bytes_temp[2] = { 0x00 };


	/* Case 1: X Axis */
	if (HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_HXL | 0x80, 1, Bytes_temp, 2, 1000) != HAL_OK) {

		return MPU9250_Read_Magnetometer_FAIL;
	}

	DataStructure->Magnetometer_X = Bytes_temp[0] << 8 | Bytes_temp[1];

	/* Case 2: Y Axis */
	if (HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_HYL | 0x80, 1, Bytes_temp, 2, 1000) != HAL_OK) {

		return MPU9250_Read_Magnetometer_FAIL;
	}

	DataStructure->Magnetometer_Y = Bytes_temp[0] << 8 | Bytes_temp[1];

	/* Case 3: Z Axis */
	if (HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, MPU9250_HZL | 0x80, 1, Bytes_temp, 2, 1000) != HAL_OK) {

		return MPU9250_Read_Magnetometer_FAIL;
	}

	DataStructure->Magnetometer_Z = Bytes_temp[0] << 8 | Bytes_temp[1];


	return MPU9250_Read_Magnetometer_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */
