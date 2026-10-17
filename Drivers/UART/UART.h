#ifndef UART_H
#define UART_H

#include "stm32f103xb.h"
#include <stdint.h>

/*
 * ============================================================================
 * UART1 DRIVER - CMSIS REGISTER + DMA + INTERRUPT
 * ============================================================================
 *
 * MCU       : STM32F103C8T6
 * USART     : USART1
 * TX pin    : PA9
 * RX pin    : PA10
 * Frame     : 8 data bit, No parity, 1 stop bit (8N1)
 *
 * DMA:
 *   - USART1_TX -> DMA1 Channel 4
 *   - USART1_RX -> DMA1 Channel 5 (Circular)
 *
 * Interrupt:
 *   - DMA1 Channel 4: TX complete / TX error
 *   - DMA1 Channel 5: RX Half Transfer / Transfer Complete / RX error
 *   - USART1         : TC va PE/FE/NE/ORE
 *
 * Main chi su dung:
 *   UART1_Init()
 *   UART1_Write()
 *   UART1_Read()
 *   UART1_IsTxIdle()
 *   UART1_GetStatus()
 *   UART1_ClearStatus()
 * ============================================================================
 */

#define UART1_RX_DMA_BUFFER_SIZE    256u
#define UART1_TX_BUFFER_SIZE        256u

#if ((UART1_RX_DMA_BUFFER_SIZE < 2u) || ((UART1_RX_DMA_BUFFER_SIZE % 2u) != 0u))
#error "UART1_RX_DMA_BUFFER_SIZE must be an even number >= 2"
#endif

#if (UART1_TX_BUFFER_SIZE < 2u)
#error "UART1_TX_BUFFER_SIZE must be >= 2"
#endif


typedef enum
{
    UART1_OK = 0,

    /* TX queue khong du cho TOAN BO block moi. */
    UART1_BUSY,

    /* Tham so NULL, length = 0, baudrate khong hop le... */
    UART1_ERROR_PARAM,

    /* API duoc goi truoc UART1_Init(). */
    UART1_ERROR_NOT_INITIALIZED,

    /* Clock APB2 khong hop le. */
    UART1_ERROR_CLOCK,

    /* Main doc RX khong kip, DMA da ghi de du lieu chua doc. */
    UART1_ERROR_RX_OVERFLOW,

    /* DMA1 Channel 4 Transfer Error. */
    UART1_ERROR_DMA_TX,

    /* DMA1 Channel 5 Transfer Error. */
    UART1_ERROR_DMA_RX,

    /* USART1 PE / FE / NE / ORE. */
    UART1_ERROR_HW

} UART1_Status_t;


/*
 * Bitmask chan doan chi tiet loi USART.
 */
#define UART1_HWERR_NONE    0x00000000u
#define UART1_HWERR_PE      0x00000001u
#define UART1_HWERR_FE      0x00000002u
#define UART1_HWERR_NE      0x00000004u
#define UART1_HWERR_ORE     0x00000008u


/*
 * Khoi tao UART1 + DMA + NVIC.
 *
 * Yeu cau:
 *   SystemClock_Config() da chay truoc ham nay.
 *
 * Driver tu:
 *   - cau hinh PA9/PA10,
 *   - tinh PCLK2 va BRR,
 *   - cau hinh DMA TX/RX,
 *   - bat RX DMA Circular,
 *   - bat cac IRQ can thiet.
 */
UART1_Status_t UART1_Init(uint32_t baudrate);


/*
 * Ghi TOAN BO block vao TX circular queue.
 *
 * UART1_OK:
 *   Driver da giu ban sao cua toan bo data.
 *
 * UART1_BUSY:
 *   TX queue khong du cho toan bo block.
 *   Driver khong nhan byte nao.
 */
UART1_Status_t UART1_Write(
    const uint8_t *data,
    uint16_t length
);


/*
 * Lay toi da capacity byte moi tu RX DMA Circular.
 *
 * Non-blocking:
 *   0  -> chua co du lieu moi
 *   >0 -> so byte thuc te da copy vao dst
 */
uint16_t UART1_Read(
    uint8_t *dst,
    uint16_t capacity
);


/*
 * Tra 1 khi:
 *   - TX queue rong
 *   - DMA TX khong active
 *   - USART1 TC = 1
 *
 * Tuc byte cuoi da ra het chan PA9.
 */
uint8_t UART1_IsTxIdle(void);


/*
 * Lay trang thai/loi sticky hien tai.
 */
UART1_Status_t UART1_GetStatus(void);


/*
 * Xoa status va bitmask loi sau khi main da ghi nhan.
 *
 * Khong xoa RX data/TX data hop le dang con trong driver.
 */
void UART1_ClearStatus(void);


/*
 * Lay bitmask PE/FE/NE/ORE.
 */
uint32_t UART1_GetHwErrorFlags(void);


/*
 * Logic IRQ cua module.
 *
 * stm32f1xx_it.c can co:
 *
 * void DMA1_Channel4_IRQHandler(void)
 * {
 *     UART1_DMA_TX_IRQHandler();
 * }
 *
 * void DMA1_Channel5_IRQHandler(void)
 * {
 *     UART1_DMA_RX_IRQHandler();
 * }
 *
 * void USART1_IRQHandler(void)
 * {
 *     UART1_IRQHandler();
 * }
 */
void UART1_DMA_TX_IRQHandler(void);
void UART1_DMA_RX_IRQHandler(void);
void UART1_IRQHandler(void);


/*
 * Weak callback.
 *
 * Application co the override trong main.c.
 */
void UART1_TxCompleteCallback(void);
void UART1_ErrorCallback(UART1_Status_t error);


#endif /* UART_H */
