/*
 * crc.c
 *
 *  Created on: 31.05.2019
 *      Author: mnowak5
 */

#include "crc.h"

byte CRC8_SingleByte(byte CRC_prev, byte Data) {

	CRC_prev ^= Data;

	for(byte Idx = 0; Idx < 8; ++Idx) {

		if( (CRC_prev & 0x80) != 0 ) {

			CRC_prev = (byte)( (CRC_prev << 1) ^ POLYNOMIAL_9 );
		} else {

			CRC_prev = (byte)(CRC_prev << 1);
		}
	}

	return CRC_prev;
}

byte CRC8_DataArray(byte *pData, byte Len) {

	byte CRC_final = 0xFF;

	for(byte Idx = 0; Idx < Len; ++Idx) {

		CRC_final = CRC8_SingleByte(CRC_final, pData[Idx]);
	}

	return CRC_final;
}
