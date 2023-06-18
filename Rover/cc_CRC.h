#ifndef __CC_CRC_H__
#define __CC_CRC_H__

#include "Rover.h"

uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen );
void generateCRC(uint8_t *buffer);

#endif