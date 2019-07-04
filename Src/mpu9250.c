/*
 * mpu9250.c
 *
 *  Created on: 04.07.2019
 *  Author: mnowak5
 */

#include "mpu9250.h"

uint8_t Address_temp = 0x00;

MPU9250_Error_code MPU9250_Init(I2C_HandleTypeDef *I2Cx,
								struct MPU9250 *DataStructure,
								MPU9250_Device_number Number) {

	uint8_t Byte_temp;

	DataStructure->Device_addres = (0x68 | Number) << 1;

	if( HAL_I2C_IsDeviceReady(I2Cx, DataStructure->Device_addres, 1, 1000) != HAL_OK ) {

		return MPU9250_Init_FAIL;
	}

	if( HAL_I2C_Mem_Read(I2Cx, DataStructure->Device_addres, WHO_AM_I, 1, &Byte_temp, 1, 1000) != HAL_OK ) {

		return MPU9250_Init_FAIL;
	}

	if( Byte_temp != 0x71 ) {

		return MPU9250_Init_FAIL;
	}

	return MPU9250_Init_OK;
}
