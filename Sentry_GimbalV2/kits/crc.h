#ifndef CRC_H
#define CRC_H

#include "typedef.h"

void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes);

uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint8_t ucCRC8);
uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength) ;

#endif
