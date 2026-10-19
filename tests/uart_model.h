#ifndef UART_DMA_HOST_MODEL_H
#define UART_DMA_HOST_MODEL_H
/* Peripheral model only: checks driver ownership/framing, not ARM timing. */
#include "mcu_mock.h"
#include "UART.h"
static uintptr_t model_tx_address;
static uint32_t model_tx_length;
static void model_uart_receive(const uint8_t *bytes, uint32_t count, uint8_t service_irq)
{
    uint32_t i;
    for (i = 0U; i < count; ++i) {
        uint32_t position;
        assert(mock_dma5.CCR & DMA_CCR_EN);
        position = UART1_RX_DMA_BUFFER_SIZE - mock_dma5.CNDTR;
        ((volatile uint8_t *)mock_dma5.CMAR)[position] = bytes[i];
        --mock_dma5.CNDTR;
        if (mock_dma5.CNDTR == UART1_RX_DMA_BUFFER_SIZE / 2U) mock_dma.ISR |= DMA_ISR_HTIF5;
        if (mock_dma5.CNDTR == 0U) {
            mock_dma.ISR |= DMA_ISR_TCIF5;
            mock_dma5.CNDTR = UART1_RX_DMA_BUFFER_SIZE;
        }
        if (service_irq && !mock_primask) UART1_DMA_RX_IRQHandler();
    }
}
static uint32_t model_uart_transmit(uint8_t *out, uint32_t capacity)
{
    uint32_t count = 0U;
    while (count < capacity && (mock_dma4.CCR & DMA_CCR_EN) && mock_dma4.CNDTR) {
        if (!model_tx_address) {
            model_tx_address = mock_dma4.CMAR;
            model_tx_length = mock_dma4.CNDTR;
        }
        mock_usart.SR &= ~USART_SR_TC;
        out[count++] = ((const uint8_t *)model_tx_address)[model_tx_length - mock_dma4.CNDTR];
        if (--mock_dma4.CNDTR == 0U) {
            model_tx_address = 0U;
            mock_dma.ISR |= DMA_ISR_TCIF4;
            UART1_DMA_TX_IRQHandler();
        }
    }
    if (!(mock_dma4.CCR & DMA_CCR_EN)) {
        mock_usart.SR |= USART_SR_TC;
        UART1_IRQHandler();
    }
    return count;
}
#endif
