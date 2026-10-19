#ifndef TRANSFER_TEST_MCU_MOCK_H
#define TRANSFER_TEST_MCU_MOCK_H
#include "stm32f1xx.h"
extern USART_TypeDef mock_usart;
extern GPIO_TypeDef mock_gpio;
extern RCC_TypeDef mock_rcc;
extern AFIO_TypeDef mock_afio;
extern TIM_TypeDef mock_tim;
extern DWT_Type mock_dwt;
extern CoreDebug_Type mock_core_debug;
typedef struct {
    volatile uint32_t CCR, CNDTR;
    volatile uintptr_t CPAR, CMAR;
} MockDMAChannel;
extern DMA_TypeDef mock_dma;
extern MockDMAChannel mock_dma4, mock_dma5;
extern uint32_t mock_primask, mock_irq_enabled;
#undef USART1
#undef GPIOA
#undef RCC
#undef AFIO
#undef TIM3
#undef DWT
#undef CoreDebug
#undef __DMB
#undef __get_PRIMASK
#undef __set_PRIMASK
#undef __disable_irq
#undef __enable_irq
#undef DMA1
#undef DMA1_Channel4
#undef DMA1_Channel5
#undef NVIC_EnableIRQ
#undef NVIC_DisableIRQ
#undef NVIC_SetPriority
#undef NVIC_ClearPendingIRQ
#define USART1 (&mock_usart)
#define GPIOA (&mock_gpio)
#define RCC (&mock_rcc)
#define AFIO (&mock_afio)
#define TIM3 (&mock_tim)
#define DWT (&mock_dwt)
#define CoreDebug (&mock_core_debug)
#define DMA1 (&mock_dma)
#define DMA1_Channel4 (&mock_dma4)
#define DMA1_Channel5 (&mock_dma5)
static inline void mock_dma_clear_flags(uint32_t flags)
{
    mock_dma.IFCR = flags;
    if (flags & DMA_IFCR_CGIF4) flags |= 0xFU << 12U;
    if (flags & DMA_IFCR_CGIF5) flags |= 0xFU << 16U;
    mock_dma.ISR &= ~flags;
}
#define UART1_CLEAR_DMA_FLAGS(flags) mock_dma_clear_flags(flags)
#define __DMB() ((void)0)
#define __get_PRIMASK() mock_primask
#define __set_PRIMASK(v) (mock_primask = (v))
#define __disable_irq() (mock_primask = 1U)
#define __enable_irq() (mock_primask = 0U)
#define NVIC_EnableIRQ(v) ((void)(v), mock_irq_enabled = 1U)
#define NVIC_DisableIRQ(v) ((void)(v), mock_irq_enabled = 0U)
#define NVIC_SetPriority(v, p) ((void)(v), (void)(p))
#define NVIC_ClearPendingIRQ(v) ((void)(v))
#endif
