#include "chassis_communication.h"
#include "referee_system_communication.h"
#include "communication.h"
#include "car.h"
#include "crc.h"
chassis_info_t rx_chassis_info;
chassis_info_t chassis_info;
uint8_t tx_buf[256];
extern car_t car;
void decode_chassis(uint8_t *data){
	memcpy(&chassis_info, data, CHASSIS_INFO_LEN);
}

//TODO: you should the method how to decode diff kinds msg, merge decode_chassis() and decode_recv_data()
//additionally, this function should be part of communication rather than chassis_communication, since the function of this method is process all communication from diff devices
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

void encode_send_data(UART_HandleTypeDef *huart, uint8_t* data, uint8_t cmd_id, uint8_t len){
	uint8_t frame_len = len + MSG_LEN_HEADER + MSG_LEN_TAIL;
	tx_buf[0] = MSG_SOF_BOARD;
	tx_buf[1] = frame_len;
	tx_buf[2] = cmd_id;
	
	//tx_buf[3]
	Append_CRC8_Check_Sum(tx_buf,MSG_LEN_HEADER);
	
	//data
	memcpy(&tx_buf[MSG_LEN_HEADER], data, len);
	tx_buf[MSG_LEN_HEADER + len] = 0xFE;
	Append_CRC16_Check_Sum(tx_buf, frame_len);
	
	uart_tx(huart, tx_buf,frame_len);
}
