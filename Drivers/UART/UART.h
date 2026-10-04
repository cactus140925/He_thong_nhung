#ifndef UART_H
#define UART_H

#include "stm32f103xb.h"
#include <stdint.h>

/* USART1 nam tr�n APB2, PCLK2 = 72 MHz.
 */
#define UART1_PCLK_HZ 72000000UL

//khoi tao USART1
void UART1_Init(uint32_t baudrate);

/* C�c h�m co ban - polling */
void UART1_SendChar(char c);
void UART1_SendString(const char *str);
void UART1_SendBufferBlocking(const uint8_t *data, uint16_t length);

uint8_t UART1_DataAvailable(void);
uint8_t UART1_ReadByte(void);

/* C�c h�m DMA */
void UART1_DMA_Init(void);
uint8_t UART1_SendDMA(const uint8_t *data, uint16_t length);
void UART1_StartRxDMA(uint8_t *buffer, uint16_t length);
uint16_t UART1_RxDMA_GetPosition(uint16_t buffer_size);
uint8_t UART1_TxDMA_Busy(void);


/* Callback khi DMA goi xong hocc loi */
void UART1_TxCompleteCallback(void);
void UART1_TxErrorCallback(void);

#endif
