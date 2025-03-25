#include "communication.h"
#include "imu_protocol.h"
#include "bsp_rc.h"
#include "bsp_uart.h"
#include "crc.h"
#include <stdarg.h>
#include <stdio.h>
#include "auto_aim.h"
#define RX_BUF_LEN 128
#define IMU_FRAME_HEADER_LEN 6
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
 
uint8_t rx1Data[RX_BUF_LEN];
uint8_t rx6Data[RX_BUF_LEN];
uint16_t rx1_len;
uint16_t rx6_len;

uint16_t templen;
uint8_t tempbuf[RX_BUF_LEN];

void uart_dma_rx_enable(void){	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	__HAL_DMA_ENABLE(&hdma_usart1_rx);
	HAL_UART_Receive_DMA(&huart1, rx1Data, rx1_len);
	
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	__HAL_DMA_ENABLE(&hdma_usart6_rx);
	HAL_UART_Receive_DMA(&huart6, rx6Data, rx6_len);
}

void uart_tx(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t size){
	HAL_UART_Transmit_DMA(huart, pData, size);
}

void uart1_printf(const char *fmt,...)
{
	static uint8_t tx_buf[256] = {0};
	static va_list ap;
	static uint16_t len;
	va_start(ap, fmt);
	
	len = vsprintf((char* )tx_buf, fmt, ap);
	
	va_end(ap);
	
	HAL_UART_Transmit(&huart1, tx_buf, len, 100);
	uart1_tx_dma_enable(tx_buf, len);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if( huart->ErrorCode & HAL_UART_ERROR_ORE )//Overflow error
   {
       uint32_t temp = huart->Instance->SR;
       temp = huart->Instance->DR;
   }
}

void USART1_IRQHandler(void)
{
	for(int i=0;i<RX_BUF_LEN;i++)
		{
		if(rx1Data[i] == MSG_SOF1_AUTO_AIM && rx1Data[i+17] == MSG_EOF_AUTO_AIM)
		{
			uint8_t header_crc_flag = Verify_CRC8_Check_Sum(&rx1Data[i], MSG_LEN_HEADER_AUTO_AIM);
			if(header_crc_flag){
				uint8_t frame_len = rx1Data[i+2];
				uint8_t frame_crc_flag = Verify_CRC16_Check_Sum(&rx1Data[i], frame_len);
				if(frame_crc_flag){
					decode_recv_data(&rx1Data[i]);
				}
			}
		}
		}
	USER_UART_IRQHandler(huart1, hdma_usart1_rx, rx1Data, rx1_len);
}

void USART6_IRQHandler(void)
{
	for(int i=0;i<RX_BUF_LEN;i++)
	{
		if(rx6Data[i]==0x5A&&rx6Data[i+1]==0xA5)
		{
			uint16_t payload_len = (uint16_t)rx6Data[i+3]<<8 | rx6Data[i+2];
			uint16_t crc;
			crc=0;
			templen = payload_len;
		
			/* calulate 5A A5 and LEN filed crc */
			crc16_update(&crc, &rx6Data[i], 4);
			/* calulate payload crc */
			crc16_update(&crc, &rx6Data[i] + 6, payload_len);
			
			if(crc == ((uint16_t)rx6Data[i+5]<<8 | rx6Data[i+4]))
				imu_decode(&rx6Data[i]);
		}
	}
	USER_UART_IRQHandler(huart6, hdma_usart6_rx, rx6Data, rx6_len);
}
