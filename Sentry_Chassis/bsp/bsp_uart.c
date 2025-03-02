/**
 * encoding:UTF-8
 * @file bsp_usart.c
 * @author Brandon
 * @brief  �����Ѳ��ԣ�û���κ�����
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     Dec-26-2018     Brandon         1. done usart6 tx
 * 
 */

#include "main.h"
#include "bsp_uart.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;

void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
	__HAL_DMA_DISABLE(&hdma_usart6_tx);
	
	
	while(hdma_usart6_tx.Instance ->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_tx);
	}
	
	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx,DMA_HISR_TCIF6);
	
	hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
	__HAL_DMA_SET_COUNTER(&hdma_usart6_tx , len);
	
	__HAL_DMA_ENABLE(&hdma_usart6_tx);
}

void uart7_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	SET_BIT(huart7.Instance->CR3, USART_CR3_DMAT);	
	SET_BIT(huart7.Instance->CR3, USART_CR3_DMAR);
	
	__HAL_DMA_DISABLE(&hdma_uart7_tx);
	
}

void uart8_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	SET_BIT(huart8.Instance->CR3, USART_CR3_DMAT);
	SET_BIT(huart8.Instance->CR3, USART_CR3_DMAR);
	//���÷���
	__HAL_DMA_DISABLE(&hdma_uart8_tx);
	
	while(hdma_uart8_tx.Instance->CR & DMA_SxCR_EN){
	  __HAL_DMA_DISABLE(&hdma_uart8_tx);
	}
	
	hdma_uart8_tx.Instance->PAR = (uint32_t)&(UART8->DR);
	hdma_uart8_tx.Instance->M0AR=(uint32_t)(NULL);
	hdma_usart6_tx.Instance->NDTR = 0;
	
	//enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);



    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_uart8_rx);
    
    while(hdma_uart8_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart8_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx, DMA_LISR_TCIF1);

    hdma_uart8_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_uart8_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    hdma_uart8_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    __HAL_DMA_SET_COUNTER(&hdma_uart8_rx, dma_buf_num);

    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_uart8_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_uart8_rx);


    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_uart8_rx);

    while(hdma_uart8_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart8_rx);
    }

    hdma_uart8_rx.Instance->PAR = (uint32_t) & (UART8->DR);
}
//����ʹ�õ�dma������Ż�
void uart8_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_uart8_tx);

    while(hdma_uart8_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart8_tx);
    }
		//
    __HAL_DMA_CLEAR_FLAG(&hdma_uart8_tx,DMA_LISR_TCIF0);
    hdma_uart8_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_uart8_tx, len);

    __HAL_DMA_ENABLE(&hdma_uart8_tx);
}

void uart7_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_uart7_tx);

    while(hdma_uart7_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart7_tx);
    }
		//
    __HAL_DMA_CLEAR_FLAG(&hdma_uart7_tx,DMA_LISR_TCIF1);
    hdma_uart7_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_uart7_tx, len);

    __HAL_DMA_ENABLE(&hdma_uart7_tx);
}
