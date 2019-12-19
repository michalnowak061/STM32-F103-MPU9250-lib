/*
 * mpu9250.h
 *
 *  Created on: 04.07.2019
 *  Author: mnowak5
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "i2c.h"
#include "math.h"

#include "quaternion.h"

#include "kalman.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"

#define MAGNETIC_DECLINATION	( 4 + (29 / 60) ) /* <- for Boleslawiec */

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

#define MPU9250_SELF_TEST_X_GYRO	0x00
#define MPU9250_SELF_TEST_Y_GYRO	0x01
#define MPU9250_SELF_TEST_Z_GYRO	0x02
#define MPU9250_SELF_TEST_X_ACCEL	0x0D
#define MPU9250_SELF_TEST_Y_ACCEL	0x0E
#define MPU9250_SELF_TEST_Z_ACCEL	0x0F
#define MPU9250_XG_OFFSET_H 		0x13
#define MPU9250_XG_OFFSET_L 		0x14
#define MPU9250_YG_OFFSET_H 		0x15
#define MPU9250_YG_OFFSET_L			0x16
#define MPU9250_ZG_OFFSET_H 		0x17
#define MPU9250_ZG_OFFSET_L			0x18
#define MPU9250_SMPLRT_DIV			0x19
#define MPU9250_CONFIG				0x1A
#define MPU9250_GYRO_CONFIG			0x1B
#define MPU9250_ACCEL_CONFIG		0x1C
#define MPU9250_ACCEL_CONFIG_2		0x1D
#define MPU9250_LP_ACCEL_ODR		0x1E
#define MPU9250_WOM_THR 			0x1F
#define MPU9250_FIFO_EN				0x23
#define MPU9250_I2C_MST_CTRL		0x24
#define MPU9250_I2C_SLV0_ADDR		0x25
#define MPU9250_I2C_SLV0_REG		0x26
#define MPU9250_I2C_SLV0_CTRL		0x27
#define MPU9250_I2C_SLV1_ADDR		0x28
#define MPU9250_I2C_SLV1_REG		0x29
#define MPU9250_I2C_SLV1_CTRL 		0x2A
#define MPU9250_I2C_SLV2_ADDR		0x2B
#define MPU9250_I2C_SLV2_REG		0x2C
#define MPU9250_I2C_SLV2_CTRL		0x2D
#define MPU9250_I2C_SLV3_ADDR		0x2E
#define MPU9250_I2C_SLV3_REG		0x2F
#define MPU9250_I2C_SLV3_CTRL		0x30
#define MPU9250_I2C_SLV4_ADDR		0x31
#define MPU9250_I2C_SLV4_REG		0x32
#define MPU9250_I2C_SLV4_DO			0x33
#define MPU9250_I2C_SLV4_CTRL		0x34
#define MPU9250_I2C_SLV4_DI			0x35
#define MPU9250_I2C_MST_STATUS		0x36
#define MPU9250_INT_PIN_CFG 		0x37
#define MPU9250_INT_ENABLE			0x38
#define MPU9250_INT_STATUS			0x3A
#define MPU9250_ACCEL_XOUT_H		0x3B
#define MPU9250_ACCEL_XOUT_L		0x3C
#define MPU9250_ACCEL_YOUT_H		0x3D
#define MPU9250_ACCEL_YOUT_L		0x3E
#define MPU9250_ACCEL_ZOUT_H 		0x3F
#define MPU9250_ACCEL_ZOUT_L		0x40
#define MPU9250_TEMP_OUT_H			0x41
#define MPU9250_TEMP_OUT_L			0x42
#define MPU9250_GYRO_XOUT_H 		0x43
#define MPU9250_GYRO_XOUT_L			0x44
#define MPU9250_GYRO_YOUT_H			0x45
#define MPU9250_GYRO_YOUT_L			0x46
#define	MPU9250_GYRO_ZOUT_H			0x47
#define MPU9250_GYRO_ZOUT_L			0x48
#define MPU9250_EXT_SENS_DATA_00	0x49
#define MPU9250_EXT_SENS_DATA_01	0x4A
#define MPU9250_EXT_SENS_DATA_02	0x4B
#define MPU9250_EXT_SENS_DATA_03	0x4C
#define MPU9250_EXT_SENS_DATA_04	0x4D
#define MPU9250_EXT_SENS_DATA_05	0x4E
#define MPU9250_EXT_SENS_DATA_06	0x4F
#define MPU9250_EXT_SENS_DATA_07	0x50
#define MPU9250_EXT_SENS_DATA_08	0x51
#define MPU9250_EXT_SENS_DATA_09	0x52
#define MPU9250_EXT_SENS_DATA_10	0x53
#define MPU9250_EXT_SENS_DATA_11	0x54
#define MPU9250_EXT_SENS_DATA_12	0x55
#define MPU9250_EXT_SENS_DATA_13	0x56
#define MPU9250_EXT_SENS_DATA_14	0x57
#define MPU9250_EXT_SENS_DATA_15	0x58
#define MPU9250_EXT_SENS_DATA_16	0x59
#define MPU9250_EXT_SENS_DATA_17	0x5A
#define MPU9250_EXT_SENS_DATA_18	0x5B
#define MPU9250_EXT_SENS_DATA_19	0x5C
#define MPU9250_EXT_SENS_DATA_20	0x5D
#define MPU9250_EXT_SENS_DATA_21	0x5E
#define MPU9250_EXT_SENS_DATA_22	0x5F
#define MPU9250_EXT_SENS_DATA_23	0x60
#define MPU9250_I2C_SLV0_DO			0x63
#define MPU9250_I2C_SLV1_DO			0x64
#define MPU9250_I2C_SLV2_DO			0x65
#define MPU9250_I2C_SLV3_DO			0x66
#define MPU9250_I2C_MST_DELAY_CTRL	0x67
#define MPU9250_SIGNAL_PATH_RESET	0x68
#define MPU9250_MOT_DETECT_CTRL		0x69
#define MPU9250_USER_CTRL			0x6A
#define MPU9250_PWR_MGMT_1			0x6B
#define MPU9250_PWR_MGMT_2			0x6C
#define MPU9250_FIFO_COUNTH			0x72
#define MPU9250_FIFO_COUNTL			0x73
#define MPU9250_FIFO_R_W 			0x74
#define MPU9250_WHO_AM_I			0x75
#define MPU9250_XA_OFFSET_H			0x77
#define MPU9250_XA_OFFSET_L			0x78
#define MPU9250_YA_OFFSET_H			0x7A
#define MPU9250_YA_OFFSET_L			0x7B
#define MPU9250_ZA_OFFSET_H			0x7D
#define MPU9250_ZA_OFFSET_L			0x7E

#define AK9863_WIA					0x00
#define AK9863_INFO					0x01
#define AK9863_ST1					0x02
#define AK9863_HXL					0x03
#define AK9863_HXH					0x04
#define AK9863_HYL					0x05
#define	AK9863_HYH					0x06
#define AK9863_HZL					0x07
#define AK9863_HZH					0x08
#define AK9863_ST2					0x09
#define AK9863_CNTL1				0x0A
#define AK9863_CNTL2				0x0B
#define AK9863_RSV					0x0B
#define AK9863_ASTC					0x0C
#define AK9863_TS1					0x0D
#define AK9863_TS2					0x0E
#define AK9863_I2CDIS				0x0F
#define AK9863_ASAX					0x10
#define AK9863_ASAY					0x11
#define AK9863_ASAZ					0x12

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

#define MPU9250_ACCE_SENSITIVITY_FACTOR_2G		16384
#define MPU9250_ACCE_SENSITIVITY_FACTOR_4G		8192
#define MPU9250_ACCE_SENSITIVITY_FACTOR_8G		4096
#define MPU9250_ACCE_SENSITIVITY_FACTOR_16G		2048

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

#define MPU9250_GYRO_SENSITIVITY_FACTOR_250s	131
#define MPU9250_GYRO_SENSITIVITY_FACTOR_500s	65.5
#define MPU9250_GYRO_SENSITIVITY_FACTOR_1000s	32.8
#define MPU9250_GYRO_SENSITIVITY_FACTOR_2000s	16.4

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

typedef enum {

	MPU9250_Init_OK							= 0,
	MPU9250_Init_FAIL						= 1,

	MPU9250_Accelerometer_Config_OK   		= 2,
	MPU9250_Accelerometer_Config_FAIL 		= 3,

	MPU9250_Gyroscope_Config_OK   			= 4,
	MPU9250_Gyroscope_Config_FAIL 			= 5,

	MPU9250_Magnetometer_Config_OK   		= 6,
	MPU9250_Magnetometer_Config_FAIL 		= 7,

	MPU9250_Read_Accelerometer_OK			= 14,
	MPU9250_Read_Accelerometer_FAIL			= 15,

	MPU9250_Read_Gyroscope_OK				= 16,
	MPU9250_Read_Gyroscope_FAIL 			= 17,

	MPU9250_Read_Magnetometer_OK   			= 18,
	MPU9250_Read_Magnetometer_FAIL 			= 19,

	MPU9250_Calib_OK   						= 10,
	MPU9250_Calib_FAIL 						= 11,

} MPU9250_Error_code;

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

typedef enum {

	MPU9250_Device_1	= 0x00,	/* ADD pin set to LOW  */
	MPU9250_Device_2	= 0x01	/* ADD pin set to HIGH */

} MPU9250_Device_number;

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

typedef enum {

	MPU9250_Acce_2G		= 0x00,
	MPU9250_Acce_4G		= 0x01,
	MPU9250_Acce_8G		= 0x02,
	MPU9250_Acce_16G	= 0x03

} MPU9250_Acce_range;

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

typedef enum {

	MPU9250_Gyro_250s	= 0x00,
	MPU9250_Gyro_500s	= 0x01,
	MPU9250_Gyro_1000s	= 0x02,
	MPU9250_Gyro_2000s	= 0x03

} MPU9250_Gyro_range;

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

struct MPU9250 {

	MPU9250_Device_number Device_number;
	uint8_t Device_addres;
	uint8_t Magnetometer_addres;
	float Magnetic_declination;

	int16_t Accelerometer_sensitivity_factor;
	float Magnetometer_sesitivity_factor;
	float Magnetometer_ASAX, Magnetometer_ASAY, Magnetometer_ASAZ;

	int16_t Accelerometer_X, Accelerometer_Y, Accelerometer_Z;
	int16_t Magnetometer_X, Magnetometer_Y, Magnetometer_Z;

	float Accelerometer_X_offset, Accelerometer_Y_offset, Accelerometer_Z_offset;
	float Magnetometer_X_offset, Magnetometer_Y_offset, Magnetometer_Z_offset;
	float Magnetometer_X_scale, Magnetometer_Y_scale, Magnetometer_Z_scale;
	float Magnetometer_Yaw_offset;

	float Accelerometer_X_g, Accelerometer_Y_g, Accelerometer_Z_g;
	float Magnetometer_X_uT,  Magnetometer_Y_uT,  Magnetometer_Z_uT;

	float Accelerometer_Roll, Accelerometer_Pitch;
	float Gyroscope_Roll, Gyroscope_Pitch, Gyroscope_Yaw;
	float Magnetometer_Roll, Magnetometer_Pitch, Magnetometer_Yaw;

	/* Gyroscope variables */
	int16_t Gyroscope_sensitivity_factor;
	int16_t Gyroscope_X, Gyroscope_Y, Gyroscope_Z;
	double Gyroscope_X_dgs, Gyroscope_Y_dgs, Gyroscope_Z_dgs;
	double Gyroscope_X_offset, Gyroscope_Y_offset, Gyroscope_Z_offset;
	struct quaternion Gyroscope_quaternion;
	struct quaternion Gyroscope_quaternion_dot;
	struct euler Gyroscope_euler;


	/* Complementary filter variables */
	float Complementary_filter_Roll, Complementary_filter_Pitch, Complementary_filter_Yaw;

	/* Kalman filter variables */
	struct Kalman Kalman_R, Kalman_P, Kalman_Y;
	float Kalman_filter_Roll, Kalman_filter_Pitch, Kalman_filter_Yaw;

	/* Madgwick filter variables */
	float Madgwick_filter_Roll, Madgwick_filter_Pitch, Madgwick_filter_Yaw;

	/* Mahony filter variables */
	float Mahony_filter_Roll, Mahony_filter_Pitch, Mahony_filter_Yaw;
};

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

MPU9250_Error_code MPU9250_Accelerometer_Configuration(I2C_HandleTypeDef *I2Cx,
													   struct MPU9250 *DataStructure,
													   MPU9250_Acce_range Range);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

MPU9250_Error_code MPU9250_Gyroscope_Configuration(I2C_HandleTypeDef *I2Cx,
												   struct MPU9250 *DataStructure,
												   MPU9250_Gyro_range Range);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */


MPU9250_Error_code MPU9250_Magnetometer_Configuration(I2C_HandleTypeDef *I2Cx,
												      struct MPU9250 *DataStructure);

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

MPU9250_Error_code MPU9250_Init(I2C_HandleTypeDef *I2Cx,
								struct MPU9250 *DataStructure,
								MPU9250_Device_number Number,
								MPU9250_Acce_range Acce_range,
								MPU9250_Gyro_range Gyro_range);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

MPU9250_Error_code MPU9250_Read_Accelerometer(I2C_HandleTypeDef *I2Cx,
											  struct MPU9250 *DataStructure);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

MPU9250_Error_code MPU9250_Read_Gyroscope(I2C_HandleTypeDef *I2Cx,
										  struct MPU9250 *DataStructure);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

MPU9250_Error_code MPU9250_Read_Magnetometer(I2C_HandleTypeDef *I2Cx,
										     struct MPU9250 *DataStructure);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Calibration_Acce(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	        struct MPU9250 *DataStructure);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Calibration_Gyro(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	        struct MPU9250 *DataStructure);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Calibration_Mag(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  	        struct MPU9250 *DataStructure);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Set_Offsets(I2C_HandleTypeDef *I2Cx,
	  	  	  	   	   	   	   	   	   struct MPU9250 *DataStructure,
									   float Acce_X_offset, float Acce_Y_offset, float Acce_Z_offset,
									   float Gyro_X_offset, float Gyro_Y_offset, float Gyro_Z_offset,
									   float Mag_X_offset, float Mag_Y_offset, float Mag_Z_offset,
									   float Mag_X_scale, float Mag_Y_scale, float Mag_Z_scale);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void MPU9250_Calculate_RPY(I2C_HandleTypeDef *I2Cx,
	      	  	  	  	  	  	  		 struct MPU9250 *DataStructure,
										 double dt);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void Complementary_filter(struct MPU9250 *DataStructure,
						  float weight_Roll_Pitch,
						  float weight_Yaw,
						  float dt);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void Kalman_filter(struct MPU9250 *DataStructure,
				   float Q_Roll_Pitch, float R_Roll_Pitch,
				   float Q_Yaw, float R_Yaw,
				   float dt);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void Madgwick_filter(struct MPU9250 *DataStructure,
					 float beta,
					 float dt);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void Mahony_filter(struct MPU9250 *DataStructure,
		           float dt);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

#endif /* MPU9250_H_ */
