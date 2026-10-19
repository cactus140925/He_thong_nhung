#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mcu_mock.h"
#include "../Drivers/UART/UART.c"
#include "uart_model.h"
USART_TypeDef mock_usart;
GPIO_TypeDef mock_gpio;
RCC_TypeDef mock_rcc;
AFIO_TypeDef mock_afio;
TIM_TypeDef mock_tim;
DMA_TypeDef mock_dma;
MockDMAChannel mock_dma4, mock_dma5;
uint32_t mock_primask, mock_irq_enabled;
uint32_t SystemCoreClock = 72000000U;
void SystemCoreClockUpdate(void) { }
int main(void)
{
    uint8_t data[512], out[512];
    uint32_t i, before;
    for (i = 0U; i < sizeof(data); ++i) data[i] = (uint8_t)(i * 73U + 0x5AU);
    assert(UART1_Write(data, 1U) == UART1_ERROR_NOT_INITIALIZED);
    UART1_IRQHandler(); assert(!mock_irq_enabled);
    mock_afio.MAPR = AFIO_MAPR_SWJ_CFG_DISABLE | AFIO_MAPR_TIM2_REMAP_0 | AFIO_MAPR_USART1_REMAP;
    before = mock_afio.MAPR;
    mock_gpio.CRH = 0x44444444UL;
    assert(UART1_Init(115200U) == UART1_OK && mock_usart.BRR == 625U);
    assert(mock_afio.MAPR == ((before & ~(AFIO_MAPR_SWJ_CFG | AFIO_MAPR_USART1_REMAP)) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE));
    assert((mock_gpio.CRH & ~0xFF0UL) == (0x44444444UL & ~0xFF0UL));
    assert((mock_gpio.CRH & 0xFF0UL) == 0x8B0UL && (mock_gpio.ODR & (1U << 10U)));
    assert(mock_dma5.CNDTR == 256U && (mock_dma5.CCR & DMA_CCR_CIRC));
    assert(UART1_Write(data, 255U) == UART1_OK);
    assert(UART1_Write(data, 1U) == UART1_BUSY);
    assert(!UART1_IsTxIdle());
    assert(model_uart_transmit(out, sizeof(out)) == 255U && memcmp(data, out, 255U) == 0);
    assert(UART1_IsTxIdle());
    assert(UART1_Write(data, 192U) == UART1_OK); /* Wrapped TX split into DMA chunks. */
    assert(model_uart_transmit(out, sizeof(out)) == 192U && memcmp(data, out, 192U) == 0);
    model_uart_receive(data, 77U, 1U);
    assert(UART1_Read(out, sizeof(out)) == 77U && memcmp(data, out, 77U) == 0);
    model_uart_receive(data + 77U, 179U, 0U); /* Pending HT+TC serviced by Read. */
    assert(UART1_Read(out, sizeof(out)) == 179U && memcmp(data + 77U, out, 179U) == 0);
    assert(UART1_Read(out, sizeof(out)) == 0U); /* NDTR reload cannot fabricate a lap. */
    model_uart_receive(data, 300U, 1U);
    assert(UART1_Read(out, sizeof(out)) == 256U);
    assert(UART1_GetStatus() == UART1_ERROR_RX_OVERFLOW);
    assert(memcmp(data + 44U, out, 256U) == 0);
    UART1_ClearStatus();
    mock_usart.SR = USART_SR_ORE; UART1_IRQHandler(); mock_usart.SR = 0U;
    assert(UART1_GetStatus() == UART1_ERROR_HW && UART1_GetHwErrorFlags() == UART1_HWERR_ORE);
    UART1_ClearStatus();
    mock_primask = 1U;
    assert(UART1_Write(data, 5U) == UART1_OK && mock_primask == 1U);
    assert(UART1_Read(out, 1U) == 0U && mock_primask == 1U);
    mock_primask = 0U;
    assert(model_uart_transmit(out, sizeof(out)) == 5U);
    assert(UART1_Write(data, 5U) == UART1_OK);
    mock_dma.ISR |= DMA_ISR_TEIF4; UART1_DMA_TX_IRQHandler();
    assert(UART1_GetStatus() == UART1_ERROR_DMA_TX && !(mock_dma4.CCR & DMA_CCR_EN));
    model_tx_address = 0U;
    assert(UART1_Init(115200U) == UART1_OK);
    mock_dma.ISR |= DMA_ISR_TEIF5; UART1_DMA_RX_IRQHandler();
    assert(UART1_GetStatus() == UART1_ERROR_DMA_RX && !(mock_dma5.CCR & DMA_CCR_EN));
    SystemCoreClock = 8000000U;
    assert(UART1_Init(115200U) == UART1_OK && mock_usart.BRR == 69U);
    SystemCoreClock = 72000000U; mock_rcc.CFGR = RCC_CFGR_PPRE2_DIV2;
    assert(UART1_Init(115200U) == UART1_OK && mock_usart.BRR == 313U);
    assert(UART1_Init(0U) == UART1_ERROR_PARAM);
    puts("PASS UART DMA: clock, SWD, GPIO, queue/wrap, RX boundaries/overflow, DMA/HW errors, TC, PRIMASK.");
    return 0;
}
