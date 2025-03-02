/**
 * encoding:UTF-8
 * @file bsp_usart.c
 * @author Brandon
 * @brief  发送已测试，没有任何问题
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     Dec-26-2018     Brandon         1. done usart6 tx
 * 
 */

#include "main.h"
#include "bsp_uart.h"

//extern UART_HandleTypeDef huart6;
//extern DMA_HandleTypeDef hdma_usart6_rx;
//extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

//void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
//{
//	__HAL_DMA_DISABLE(&hdma_usart6_tx);
//	
//	
//	while(hdma_usart6_tx.Instance ->CR & DMA_SxCR_EN)
//	{
//		__HAL_DMA_DISABLE(&hdma_usart6_tx);
//	}
//	
//	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx,DMA_HISR_TCIF6);
//	
//	hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
//	__HAL_DMA_SET_COUNTER(&hdma_usart6_tx , len);
//	
//	__HAL_DMA_ENABLE(&hdma_usart6_tx);
//}

void uart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }
		//
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_LISR_TCIF1);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}
