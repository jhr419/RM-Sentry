#ifndef BSP_UART_H
#define BSP_UART_H
#include "typedef.h"

extern void usart6_tx_dma_init(void);
extern void usart6_tx_dma_enable(uint8_t *data,uint16_t len);
extern void uart1_tx_dma_enable(uint8_t *data, uint16_t len);
#endif
