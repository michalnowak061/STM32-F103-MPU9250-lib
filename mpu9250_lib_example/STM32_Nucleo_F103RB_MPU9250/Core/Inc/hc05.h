/*
 * hc05.h
 *
 *  Created on: 23.03.2019
 *  Author: Michal
 */

#ifndef HC05_H_
#define HC05_H_

#include "usart.h"
#include "crc.h"

#define DATA_FRAME_FROM_PC_SIZE 	10
#define DATA_FRAME_TO_PC_SIZE 		31

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
struct Data_frame_to_PC {

	/* LiPol data */
	int16_t Lipol_voltage;

	/* Complementary filter data */
	int16_t Filter_Roll, Filter_Pitch, Filter_Yaw;

	/* Engines data */
	int16_t Left_engine_speed, Right_engine_speed;

	/* IMU data */
	int16_t g_x, g_y, g_z;
	int16_t a_x, a_y, a_z;
	int16_t m_x, m_y, m_z;
};

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
struct Data_frame_from_PC {

	/* Filters data */
	int16_t Complementary_filter_weight;
	int16_t Kalman_filter_process_variance;
	int16_t Kalman_filter_measure_variance;
	int16_t Madgwick_filter_beta;
	int16_t Mahony_Kp;
	int16_t Mahony_Ki;

	/* Additional data */
	int8_t Which_filter;
};

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
uint8_t HC05_Divide_int16(int16_t _data, uint8_t _which_byte);

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
int16_t HC05_Merge_bytes(uint8_t _lower_byte, uint8_t _higher_byte);

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
void HC05_Fill_Data_frame_to_PC(struct Data_frame_to_PC *_data, uint8_t *_frame,
								int16_t LiPol_voltage,
				   			    float Filter_Roll, float Filter_Pitch, float Filter_Yaw,
								int16_t Left_engine_speed, int16_t Right_engine_speed,
								float g_x_dgs, float g_y_dgs, float g_z_dgs,
								float a_x_g, float a_y_g, float a_z_g,
								float m_x_uT, float m_y_uT, float m_z_uT);

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
int HC05_Parse_Data_frame(struct Data_frame_from_PC *_data, uint8_t *_frame);

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

#endif /* HC05_H_ */
