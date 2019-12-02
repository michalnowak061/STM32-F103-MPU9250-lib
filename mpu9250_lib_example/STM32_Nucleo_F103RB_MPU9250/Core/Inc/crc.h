/*
 * crc.h
 *
 *  Created on: 31.05.2019
 *      Author: mnowak5
 */

#ifndef CRC_H_
#define CRC_H_

#include <stm32f1xx_hal.h>

#define POLYNOMIAL_9	0x31

typedef uint8_t byte;

byte CRC8_SingleByte(byte CRC_prev, byte Data);
byte CRC8_DataArray(byte *pData, byte Len);

#endif /* CRC_H_ */
