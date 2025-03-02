#include "auto_aim.h"
#include "gimbal_control.h"
#include "communication.h"
#include "car.h"
#include "crc.h"
#include "bsp_uart.h"
#include "string.h"
aim_info_t aim_info;
extern car_t car;
uint8_t temp1[20];
uint8_t temp2[12];
union_fp32 temp32;
extern UART_HandleTypeDef huart1;

void decode_recv_data(uint8_t *data){
	uint8_t data_len = data[1] - MSG_LEN_HEADER - MSG_LEN_TAIL;
	uint8_t cmd_id = data[2];
	
	switch(cmd_id){
		case MSG_CMD_BOARD_RC:
			memcpy(car.pRC, &data[MSG_LEN_HEADER], data_len);
			break;
		case MSG_CMD_AUTO_AIM:
			memcpy(car.pAim_info, &data[MSG_LEN_HEADER], data_len);
		default:
			break;
	}
}

void auto_aim_send(){
	temp1[0] = 0xE7;
	temp1[1] = 0x7E;
	temp1[2] = 20;
	temp1[3] = 0x01;
	Append_CRC8_Check_Sum(temp1,5);
	
	temp32.data = DEGREE_TO_RAD*(fp32)(car.pImu_info->yaw.data);
	memcpy(&temp1[5], &temp32.bytes[0],4);
	
	temp32.data = DEGREE_TO_RAD*(fp32)(car.pImu_info->roll.data);
	memcpy(&temp1[9], &temp32.bytes[0],4);
	
	temp32.data = DEGREE_TO_RAD*(fp32)(car.pImu_info->roll.data);
	memcpy(&temp1[13], &temp32.bytes[0],4);
	
	temp1[17] = 0xFE;
	
	Append_CRC16_Check_Sum(temp1,20);
	uart_tx(&huart1, temp1, 20);
	
	temp2[0] = 0xE7;
	temp2[1] = 0x7E;
	temp2[2] = 12;
	temp2[3] = 0x02;
	Append_CRC8_Check_Sum(temp2,5);
	temp2[5] = 0x01;
	temp2[6] = 0x01;
	temp2[7] = 0x01;
	temp2[8] = 0x01;
	temp2[9] = 0xFE;
	Append_CRC16_Check_Sum(temp2,12);
	uart_tx(&huart1, temp2, 12);
}