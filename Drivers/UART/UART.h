#ifndef UART_H
#define UART_H

#include "stm32f103xb.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#ifndef UART1_RX_DMA_BUFFER_SIZE
#define UART1_RX_DMA_BUFFER_SIZE    256U
#endif

#ifndef UART1_TX_BUFFER_SIZE
#define UART1_TX_BUFFER_SIZE        256U
#endif

#ifndef UART1_IRQ_PRIORITY
#define UART1_IRQ_PRIORITY          6U
#endif

/*
 * RX DMA buffer:
 *
 *   - toi thieu 2 byte
 *   - toi da 65535 byte
 *   - bat buoc la so chan
 */
#if ((UART1_RX_DMA_BUFFER_SIZE < 2U) || \
     (UART1_RX_DMA_BUFFER_SIZE > 65535U) || \
     ((UART1_RX_DMA_BUFFER_SIZE % 2U) != 0U))
#error "UART1_RX_DMA_BUFFER_SIZE must be even and in range 2..65535"
#endif

/*
 * TX queue dung uint16_t lam chi so,
 * vi vay kich thuoc khong duoc vuot qua 65535.
 */
#if ((UART1_TX_BUFFER_SIZE < 2U) || \
     (UART1_TX_BUFFER_SIZE > 65535U))
#error "UART1_TX_BUFFER_SIZE must be in range 2..65535"
#endif

/*
 * Kiem tra IRQ priority hop le voi NVIC.
 * STM32F103: __NVIC_PRIO_BITS = 4
 * Tuong ung priority hop le: 0 ... 15
 */
#if (UART1_IRQ_PRIORITY >= (1U << __NVIC_PRIO_BITS))
#error "UART1_IRQ_PRIORITY is outside the valid NVIC priority range"
#endif

/* ============================================================================
 * TRANG THAI DRIVER
 * ============================================================================
 */
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

/* ============================================================================
 * CO LOI PHAN CUNG USART
 * ============================================================================
 */
#define UART1_HWERR_NONE    0x00000000u
#define UART1_HWERR_PE      0x00000001u
#define UART1_HWERR_FE      0x00000002u
#define UART1_HWERR_NE      0x00000004u
#define UART1_HWERR_ORE     0x00000008u

/* ============================================================================
 * API
 * ============================================================================
 * Khoi tao UART1 + DMA + NVIC.
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
 * Doc du lieu moi tu RX DMA Circular.
 * Non-blocking:
 *   0  -> chua co du lieu moi
 *   >0 -> so byte thuc te da copy vao dst
 */
uint16_t UART1_Read(
    uint8_t *dst,
    uint16_t capacity
);

/* Kiem tra TX da hoan tat tren duong truyen hay chua.
 * Tra 1 khi:
 *   - TX queue rong
 *   - DMA TX khong active
 *   - USART1 TC = 1
 * Tuc byte cuoi da ra het chan PA9.
 */
uint8_t UART1_IsTxIdle(void);

/*
 * Lay trang thai/loi sticky hien tai.
 */
UART1_Status_t UART1_GetStatus(void);

/*
 * Xoa sticky status va hardware error flags.
 * Khong xoa RX data/TX data hop le dang con trong driver.
 */
void UART1_ClearStatus(void);

/*
 * Lay bitmask PE/FE/NE/ORE loi phan cung USART1. 
 */
uint32_t UART1_GetHwErrorFlags(void);

/* ============================================================================
 * 6. CAC HAM XU LY NGAT
 * ============================================================================
*/
void UART1_DMA_TX_IRQHandler(void);
void UART1_DMA_RX_IRQHandler(void);
void UART1_IRQHandler(void);

/* ============================================================================
 * 7. CALLBACK
 * ============================================================================
 */
void UART1_TxCompleteCallback(void);
void UART1_ErrorCallback(UART1_Status_t error);

#ifdef __cplusplus
}
#endif

#endif /* UART_H */
