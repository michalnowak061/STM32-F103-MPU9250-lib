 /*
 * hc05.c
 *
 *  Created on: 23.03.2019
 *      Author: Micha�
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
								int16_t LiPol_voltage,
							    float Filter_Roll, float Filter_Pitch, float Filter_Yaw,
								int16_t Left_engine_speed, int16_t Right_engine_speed,
								float g_x_dgs, float g_y_dgs, float g_z_dgs,
								float a_x_g, float a_y_g, float a_z_g,
								float m_x_uT, float m_y_uT, float m_z_uT) {

	/* LiPol data */
	_frame[0] = HC05_Divide_int16(LiPol_voltage, 'L');
	_frame[1] = HC05_Divide_int16(LiPol_voltage, 'H');

	/* Filter data */
	_frame[2] = HC05_Divide_int16(Filter_Roll * 100, 'L');
	_frame[3] = HC05_Divide_int16(Filter_Roll * 100, 'H');

	_frame[4] = HC05_Divide_int16(Filter_Pitch * 100, 'L');
	_frame[5] = HC05_Divide_int16(Filter_Pitch * 100, 'H');

	_frame[6] = HC05_Divide_int16(Filter_Yaw * 100, 'L');
	_frame[7] = HC05_Divide_int16(Filter_Yaw * 100, 'H');

	/* Engines data */
	_frame[8] = HC05_Divide_int16(Left_engine_speed, 'L');
	_frame[9] = HC05_Divide_int16(Left_engine_speed, 'H');

	_frame[10] = HC05_Divide_int16(Right_engine_speed, 'L');
	_frame[11] = HC05_Divide_int16(Right_engine_speed, 'H');

	/* Gyroscope data */
	_frame[12] = HC05_Divide_int16(g_x_dgs * 100, 'L');
	_frame[13] = HC05_Divide_int16(g_x_dgs * 100, 'H');

	_frame[14] = HC05_Divide_int16(g_y_dgs * 100, 'L');
	_frame[15] = HC05_Divide_int16(g_y_dgs * 100, 'H');

	_frame[16] = HC05_Divide_int16(g_z_dgs * 100, 'L');
	_frame[17] = HC05_Divide_int16(g_z_dgs * 100, 'H');

	/* Accelerometer data */
	_frame[18] = HC05_Divide_int16(a_x_g * 100, 'L');
	_frame[19] = HC05_Divide_int16(a_x_g * 100, 'H');

	_frame[20] = HC05_Divide_int16(a_y_g * 100, 'L');
	_frame[21] = HC05_Divide_int16(a_y_g * 100, 'H');

	_frame[22] = HC05_Divide_int16(a_z_g * 100, 'L');
	_frame[23] = HC05_Divide_int16(a_z_g * 100, 'H');

	/* Magnetometer data */
	_frame[24] = HC05_Divide_int16(m_x_uT * 100, 'L');
	_frame[25] = HC05_Divide_int16(m_x_uT * 100, 'H');

	_frame[26] = HC05_Divide_int16(m_y_uT * 100, 'L');
	_frame[27] = HC05_Divide_int16(m_y_uT * 100, 'H');

	_frame[28] = HC05_Divide_int16(m_z_uT * 100, 'L');
	_frame[29] = HC05_Divide_int16(m_z_uT * 100, 'H');

	/* CRC */
	_frame[30] = CRC8_DataArray(_frame, DATA_FRAME_TO_PC_SIZE - 1);
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
