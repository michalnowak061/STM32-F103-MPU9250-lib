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
#define DATA_FRAME_TO_PC_SIZE 		27

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
struct Data_frame_to_PC {

	/* Complementary filter data */
	int16_t Complementary_Roll, Complementary_Pitch, Madgwick;
	int16_t Kalman_Roll, Kalman_Pitch, Kalman_Yaw;
	int16_t Madgwick_Roll, Madgwick_Pitch, Complementary_Yaw;
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
								double Complementary_q_w, double Complementary_q_x, double Complementary_q_y, double Complementary_q_z,
								double Kalman_q_w, 		  double Kalman_q_x, 		double Kalman_q_y, 		  double Kalman_q_z,
								double Madgwick_q_w, 	  double Madgwick_q_x, 		double Madgwick_q_y, 	  double Madgwick_q_z);

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
int HC05_Parse_Data_frame(struct Data_frame_from_PC *_data, uint8_t *_frame);

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

#endif /* HC05_H_ */
