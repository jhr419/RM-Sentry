#ifndef CHASSIS_COMMUNICATION_H
#define CHASSIS_COMMUNICATION_H
#include "typedef.h"
#include "usart.h"

void decode_chassis(uint8_t *data);
void encode_send_data(UART_HandleTypeDef *huart, uint8_t* data, uint8_t cmd_id, uint8_t len);
void decode_recv_data(uint8_t *data);
#endif
