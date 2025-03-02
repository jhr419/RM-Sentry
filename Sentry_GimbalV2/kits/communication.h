#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "main.h"
#include "usart.h"
#include "string.h"
#include "dma.h"

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
				
#define MSG_SOF_BOARD 0xF1
#define MSG_EOF_BOARD 0xFE
#define MSG_LEN_HEADER 	4
#define MSG_LEN_TAIL	3	
			
#define MSG_LEN_AUTO_AIM 21
#define MSG_SOF1_AUTO_AIM 0xF1
#define MSG_SOF2_AUTO_AIM 0x7E
#define MSG_EOF_AUTO_AIM 0xFE
#define MSG_LEN_HEADER_AUTO_AIM 4
#define MSG_LEN_TAIL_AUTO_AIM 3
				
//TODO: you should determine diff kinds of cmd id
typedef enum{
	MSG_IMU_INFO_ID = 0x0104,
	MSG_MOVE_CMD_ID = 0x0204,
	GIMBAL_ROTATION_ID = 0x0304,
	
	MSG_CMD_BOARD_RC = 0xC0,
	MSG_CMD_AUTO_AIM = 0xC1,
}cmd_id_e;//2		
		
void Usart_SendString(uint8_t *str);
void uart_dma_rx_enable(void);
void uart_tx(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t size);
void uart1_printf(const char *fmt,...);
#endif
