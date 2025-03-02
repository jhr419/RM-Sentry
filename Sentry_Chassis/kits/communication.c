#include "communication.h"
#include "chassis_communication.h"
#include "bsp_rc.h"
#include "bsp_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include "crc.h"
#include "car.h"
#include "referee_system_communication.h"

#define RX_BUF_LEN 128

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
 
extern chassis_info_t chassis_info;
extern car_t car;
uint8_t rx8Data[RX_BUF_LEN];
uint8_t rx7Data[RX_BUF_LEN];
uint8_t rx6Data[RX_BUF_LEN];
uint16_t rx8_len;
uint16_t rx7_len;
uint16_t rx6_len;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART7)
    {
		for(int i=0;i<RX_BUF_LEN;i++)
		{
			if(rx7Data[i]==0xA5&&rx7Data[i+1]==0x0D&&rx7Data[i+2]==0x00&&rx7Data[i+3]==0x00&&rx7Data[i+4]==0xD3)
			{
				uint8_t flag = Verify_CRC16_Check_Sum(&rx7Data[i], CHASSIS_INFO_LEN);
				if(flag)
					#ifdef CHASSIS
						decode_chassis(&rx7Data[i]);
					#endif
					//gimbal send up pc msg
					#ifdef MAIN_GIMBAL
						uart_tx(&huart8, rx7Data, CHASSIS_INFO_LEN);
					#endif
			}		
		}
	}
		
	
	else if(huart->Instance == UART8){
		
		for(int i=0;i<RX_BUF_LEN;i++)
		{
			#ifdef MAIN_GIMBAL
			//rc infomation
			if(rx8Data[i]==MSG_SOF_BOARD)
			{
				uint8_t header_crc_flag = Verify_CRC8_Check_Sum(&rx8Data[i], MSG_LEN_HEADER);
				if(header_crc_flag){
					uint8_t frame_len = rx8Data[i+1];
					uint8_t frame_crc_flag = Verify_CRC16_Check_Sum(&rx8Data[i], frame_len);
					if(frame_crc_flag){
						decode_recv_data(&rx8Data[i]);
					}
				}
			}
			#endif
			
			#ifdef CHASSIS
			//referee system
			if(rx8Data[i] == MSG_SOF_REFEREE_SYS)
			{
				uint8_t header_crc_flag = Verify_CRC8_Check_Sum(&rx8Data[i], MSG_LEN_HEADER_REFEREE_SYS);
				if(header_crc_flag){
					uint16_t frame_len = rx8Data[i+1] | rx8Data[i+2] + 5 + 2 + 2;
					uint8_t seq = rx8Data[i+3];
					uint8_t frame_crc_flag = Verify_CRC16_Check_Sum(&rx8Data[i], frame_len);
					if(frame_crc_flag){
						decode_referee_system_data(&rx8Data[i]);
					}
				}
			}
			#endif
		}
	}
	
	
	else if(huart->Instance == USART6){
		//auto aim infomation
		for(int i=0;i<RX_BUF_LEN;i++)
		{
			#ifdef MAIN_GIMBAL
			if(rx6Data[i] == MSG_SOF1_AUTO_AIM && rx6Data[i+1] == MSG_SOF2_AUTO_AIM && rx6Data[MSG_LEN_AUTO_AIM-3] == MSG_EOF_AUTO_AIM)
			{
				uint8_t header_crc_flag = Verify_CRC8_Check_Sum(&rx6Data[i], MSG_LEN_HEADER_AUTO_AIM);
				if(header_crc_flag){
					uint8_t frame_len = rx6Data[i+2];
					uint8_t frame_crc_flag = Verify_CRC16_Check_Sum(&rx6Data[i], frame_len);
					if(frame_crc_flag){
						decode_recv_data(&rx6Data[i]);
					}
				}
			}
			#endif
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    // UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
the HAL_UART_ErrorCallback could be implemented in the user file
*/
    if( huart->ErrorCode & HAL_UART_ERROR_ORE )//Overflow error
   {
       uint32_t temp = huart->Instance->SR;
       temp = huart->Instance->DR;
   }
}

void uart_dma_rx_enable(void){	
	HAL_UART_Receive_DMA(&huart8, rx8Data, RX_BUF_LEN);
	HAL_UART_Receive_DMA(&huart7, rx7Data, RX_BUF_LEN);
	HAL_UART_Receive_DMA(&huart6, rx6Data, RX_BUF_LEN);
}

void uart_tx(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t size){
	HAL_UART_Transmit_DMA(huart, pData, size);
}

void uart_printf(UART_HandleTypeDef *huart, const char *fmt,...)
{
	static uint8_t tx_buf[256] = {0};
	static va_list ap;
	static uint16_t len;
	va_start(ap, fmt);
	
	len = vsprintf((char* )tx_buf, fmt, ap);
	
	va_end(ap);
	
	HAL_UART_Transmit(huart, tx_buf, len, 100);
	uart7_tx_dma_enable(tx_buf, len);
}

void USART1_IRQHandler(void)
{
	uart_receive_handler(&huart1);//remote control
	USER_UART_IRQHandler(huart1, hdma_uart8_rx, rx8Data, rx8_len);
}

void USART8_IRQHandler(void)
{
	USER_UART_IRQHandler(huart8, hdma_uart8_rx, rx8Data, rx8_len);
}

void USART7_IRQHandler(void)
{     
	USER_UART_IRQHandler(huart7, hdma_uart7_rx, rx7Data, rx7_len);
}


void USART6_IRQHandler(void)
{
	USER_UART_IRQHandler(huart6, hdma_usart6_rx, rx6Data, rx6_len);
}
