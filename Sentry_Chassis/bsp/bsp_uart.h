#ifndef BSP_UART_H
#define BSP_UART_H
#include "typedef.h"

extern void usart6_tx_dma_init(void);
extern void usart6_tx_dma_enable(uint8_t *data,uint16_t len);
extern void uart8_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void uart8_tx_dma_enable(uint8_t *data, uint16_t len);
extern void uart7_tx_dma_enable(uint8_t *data, uint16_t len);
#endif
