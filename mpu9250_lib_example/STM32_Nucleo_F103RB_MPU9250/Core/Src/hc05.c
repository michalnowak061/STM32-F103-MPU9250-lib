 /*
 * hc05.c
 *
 *  Created on: 23.03.2019
 *      Author: Michal
 */

#include "hc05.h"

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

uint8_t HC05_Divide_int16(int16_t _data, uint8_t _which_byte) {

	if(_which_byte == 'L') {

		return (_data & 0xFF); // return lower byte
	}
	else if(_which_byte == 'H'){

		return (_data >> 8);   // return higher byte
	}

	return 0;
}

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

int16_t HC05_Merge_bytes(uint8_t _lower_byte, uint8_t _higher_byte) {

	int16_t uint16t_byte = ( (_higher_byte << 8) + _lower_byte );

	if(uint16t_byte >= 32767) {

		int16_t int_byte = uint16t_byte - ( 2 * 32768);
		return int_byte;
	}
	else  return uint16t_byte;

	return 0;
}

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void HC05_Fill_Data_frame_to_PC(struct Data_frame_to_PC *_data, uint8_t *_frame,
							    float Complementary_Roll, float Complementary_Pitch, float Complementary_Yaw,
								float Kalman_Roll, float Kalman_Pitch, float Kalman_Yaw,
								double Madgwick_q_w, double Madgwick_q_x, double Madgwick_q_y, double Madgwick_q_z) {

	/* Complementary filter data */
	_frame[0] = HC05_Divide_int16(Complementary_Roll * 100, 'L');
	_frame[1] = HC05_Divide_int16(Complementary_Roll * 100, 'H');

	_frame[2] = HC05_Divide_int16(Complementary_Pitch * 100, 'L');
	_frame[3] = HC05_Divide_int16(Complementary_Pitch * 100, 'H');

	_frame[4] = HC05_Divide_int16(Complementary_Yaw * 100, 'L');
	_frame[5] = HC05_Divide_int16(Complementary_Yaw * 100, 'H');

	/* Complementary filter data */
	_frame[6] = HC05_Divide_int16(Kalman_Roll * 100, 'L');
	_frame[7] = HC05_Divide_int16(Kalman_Roll * 100, 'H');

	_frame[8] = HC05_Divide_int16(Kalman_Pitch * 100, 'L');
	_frame[9] = HC05_Divide_int16(Kalman_Pitch * 100, 'H');

	_frame[10] = HC05_Divide_int16(Kalman_Yaw * 100, 'L');
	_frame[11] = HC05_Divide_int16(Kalman_Yaw * 100, 'H');

	/* Complementary filter data */
	_frame[12] = HC05_Divide_int16(Madgwick_q_w * 1000, 'L');
	_frame[13] = HC05_Divide_int16(Madgwick_q_w * 1000, 'H');

	_frame[14] = HC05_Divide_int16(Madgwick_q_x * 1000, 'L');
	_frame[15] = HC05_Divide_int16(Madgwick_q_x * 1000, 'H');

	_frame[16] = HC05_Divide_int16(Madgwick_q_y * 1000, 'L');
	_frame[17] = HC05_Divide_int16(Madgwick_q_y * 1000, 'H');

	_frame[18] = HC05_Divide_int16(Madgwick_q_z * 1000, 'L');
	_frame[19] = HC05_Divide_int16(Madgwick_q_z * 1000, 'H');

	_frame[20] = HC05_Divide_int16(32768, 'L');
	_frame[21] = HC05_Divide_int16(32768, 'H');

	/* CRC */
	_frame[22] = CRC8_DataArray(_frame, DATA_FRAME_TO_PC_SIZE - 1);
}

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

int HC05_Parse_Data_frame(struct Data_frame_from_PC *_data, uint8_t *_frame) {

	uint8_t Actual_CRC = 0;
	uint8_t Received_CRC = 0;

	/* Filters data */
	_data->Complementary_filter_weight = HC05_Merge_bytes(_frame[0], _frame[1]);
	_data->Kalman_filter_process_variance = HC05_Merge_bytes(_frame[2], _frame[3]);
	_data->Kalman_filter_measure_variance = HC05_Merge_bytes(_frame[4], _frame[5]);
	_data->Madgwick_filter_beta = HC05_Merge_bytes(_frame[6], _frame[7]);

	/* Additional data */
	_data->Which_filter = _frame[8];

	/* CRC test */
	Received_CRC = _frame[9];
	Actual_CRC = CRC8_DataArray(_frame, DATA_FRAME_FROM_PC_SIZE - 1);

	if( Actual_CRC != Received_CRC ) {

		return -1;
	}

	return 0;
}

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
