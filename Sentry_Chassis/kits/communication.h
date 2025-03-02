#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "main.h"
#include "usart.h"
#include "string.h"
#include "dma.h"
#include "typedef.h"

#define USER_UART_RX_ENABLE(huartx, rxBuf, bufLen)   \
        HAL_UART_DMAStop(&huartx); 									 \
        __HAL_UART_ENABLE_IT(&huartx, UART_IT_RXNE); \
        __HAL_UART_ENABLE_IT(&huartx, UART_IT_IDLE); \
        HAL_UART_Receive_DMA(&huartx, rxBuf, bufLen)

#define USER_UART_IRQHandler(huartx, hdma_usartx_rx, rxBuf, bufLen) 		 \
				uint32_t idle_flag_temp = 0; 																		 \
				uint16_t len_temp = 0;                                           \
			                                                                   \
				HAL_UART_IRQHandler(&huartx);                                    \
				                                                                 \
				idle_flag_temp = __HAL_UART_GET_FLAG(&huartx,UART_FLAG_IDLE);    \
				if(idle_flag_temp){                                              \
					__HAL_UART_CLEAR_FLAG(&huartx,UART_FLAG_IDLE);                 \
					HAL_UART_DMAStop(&huartx);                                     \
				                                                                 \
					len_temp = __HAL_DMA_GET_COUNTER(&hdma_usartx_rx);             \
					bufLen = RX_BUF_LEN - len_temp;                                \
					bufLen = 1;                                                    \
				}                                                                \
				                                                                 \
				__HAL_UART_ENABLE_IT(&huartx,UART_IT_RXNE);                      \
				HAL_UART_Receive_DMA(&huartx, rxBuf, RX_BUF_LEN)

#define MSG_SOF_REFEREE_SYS 0xA5
#define MSG_LEN_HEADER_REFEREE_SYS 5				
				
#define MSG_LEN_AUTO_AIM 21
#define MSG_SOF1_AUTO_AIM 0xE7
#define MSG_SOF2_AUTO_AIM 0x7E
#define MSG_EOF_AUTO_AIM 0xFE
#define MSG_LEN_HEADER_AUTO_AIM 5
#define MSG_LEN_TAIL_AUTO_AIM 3

				
#define MSG_SOF_BOARD 0xF1
#define MSG_EOF_BOARD 0xFE
#define MSG_LEN_HEADER 	4
#define MSG_LEN_TAIL	3

#define CHASSIS_INFO_LEN 22
#define HEADER_LEN 5
#define CMD_LEN 2
#define DATA_LEN 13
			
typedef struct __PACKED{
	uint8_t sof;		 // default a5
	uint8_t data_len;	 // default 0d
	uint8_t fixed_data1; // default 0
	uint8_t fixed_data2; // default 0
	uint8_t header_crc8; // default d3
}header_t;//5

//TODO: you should determine diff kinds of cmd id
typedef enum{
	MSG_IMU_INFO_ID = 0x0104,
	MSG_MOVE_CMD_ID = 0x0204,
	GIMBAL_ROTATION_ID = 0x0304,
	
	MSG_CMD_BOARD_RC = 0xC0,
	MSG_CMD_AUTO_AIM = 0xC1,
}cmd_id_e;//2
	
typedef struct __PACKED{
	union_fp32 speedX;
	union_fp32 speedY;
	union_fp32 speedZ;
	uint8_t top_flag; // default 0
}data_t;//7
				
typedef struct __PACKED{
	header_t header;
	uint16_t cmd_id;
	data_t data;
	uint16_t crc16;
}chassis_info_t;//22

void Usart_SendString(uint8_t *str);
void uart_dma_rx_enable(void);
void uart_tx(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t size);
void uart_printf(UART_HandleTypeDef *huart, const char *fmt,...);

#endif
