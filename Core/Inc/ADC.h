#ifndef CMSIS_ADC_MODULE_H_
#define CMSIS_ADC_MODULE_H_

#include <stdint.h>
#include "stm32f1xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Target: STM32F103C8/CB, ADC1, DMA1 Channel 1, TIM3 TRGO.
 * ADC/DMA are configured here. TIM3 belongs to your separate Timer module.
 * No HAL or LL peripheral driver functions are used.
 */
#define ADC_CH_PA0 0U
#define ADC_CH_PA1 1U
#define ADC_CH_PA2 2U
#define ADC_CH_PA3 3U
#define ADC_CH_PA4 4U
#define ADC_CH_PA5 5U
#define ADC_CH_PA6 6U
#define ADC_CH_PA7 7U
#define ADC_CH_PB0 8U
#define ADC_CH_PB1 9U
#define ADC_MAX_VALUE 4095U

#ifndef ADC_TIMEOUT_LOOPS
#define ADC_TIMEOUT_LOOPS 100000UL /* Loop budget, not milliseconds. */
#endif

/* SMPR encoding: 0=1.5, 1=7.5, 2=13.5, 3=28.5, 4=41.5,
 * 5=55.5, 6=71.5, 7=239.5 ADC clock cycles.
 * With the default and ADCCLK=12 MHz, one conversion takes 21 us.
 * A Timer trigger period MUST exceed the ADC conversion time.
 */
#ifndef ADC_SAMPLE_TIME_BITS
#define ADC_SAMPLE_TIME_BITS 7U
#endif

#ifndef ADC_DMA_IRQ_PRIORITY
#define ADC_DMA_IRQ_PRIORITY 5U
#endif

/* Default: ADC.c provides DMA1_Channel1_IRQHandler().
 * Set this to 0 project-wide if stm32f1xx_it.c owns the vector, and call
 * ADC_DMA_IRQHandler() from that vector. Define each vector only once.
 */
#ifndef ADC_DMA_DEFINE_IRQHANDLER
#define ADC_DMA_DEFINE_IRQHANDLER 1U
#endif

typedef enum
{
    ADC_OK = 0,
    ADC_ERROR_PARAM,
    ADC_ERROR_TIMEOUT,
    ADC_ERROR_NOT_INITIALIZED,
    ADC_ERROR_CLOCK,
    ADC_ERROR_BUSY,
    ADC_ERROR_DMA,
    ADC_ERROR_OVERRUN,
    ADC_NO_DATA
} ADC_Status_t;

typedef enum
{
    ADC_DMA_NORMAL = 0, /* One buffer, then automatically stop. */
    ADC_DMA_CIRCULAR    /* Continue until ADC_StopDMA() or an error. */
} ADC_DMA_Mode_t;

/* Call after SystemClock_Config(). Calibrates ADC1 and selects a default pin.
 * ADCCLK=PCLK2/6. Owns ADC1 and the shared ADC prescaler.
 * Do not change clocks or run another ADC1/DMA1 Channel 1 driver concurrently.
 * Public APIs are for one foreground context, not concurrent threads/ISRs.
 */
ADC_Status_t ADC_Init(uint8_t channel);

/* Single software-triggered conversions for testing, while DMA is idle.
 * Result is written only on ADC_OK; range 0..4095.
 * A polling/calibration timeout requires ADC_Init() before retrying.
 */
ADC_Status_t ADC_Read(uint16_t *value);
ADC_Status_t ADC_ReadChannel(uint8_t channel, uint16_t *value);

/* TIM3 must be stopped before this call; configure its update TRGO first.
 * Then start TIM3 ONLY AFTER ADC_StartDMA() returns ADC_OK.
 * buffer: writable RAM, 2-byte aligned, lifetime covering the acquisition.
 * length: even number of uint16_t samples, 2..65534 (NOT a byte count).
 * Each ready block contains length/2 samples. Normal mode stops after length.
 * CONT=0: each Timer event produces exactly one regular conversion.
 * This module never writes TIM3 registers and never starts the Timer itself.
 */
ADC_Status_t ADC_StartDMA(uint8_t channel, uint16_t *buffer,
                          uint16_t length, ADC_DMA_Mode_t mode);

/* Stop TIM3 first. Cancels DMA and discards blocks not yet copied.
 * Calibration is retained; DMA can be restarted without ADC_Init().
 */
void ADC_StopDMA(void);

/* Copy the oldest completed half-buffer into independent application RAM.
 * capacity is in samples and must be >= length/2; destination must not overlap
 * the DMA buffer. Only use destination/count/index when ADC_OK is returned.
 * ADC_NO_DATA means try later; no waiting occurs.
 * first_sample_index is zero-based within this DMA session.
 * Circular mode: call before the next half-buffer completes. If the deadline
 * is missed (or missed during copying), capture stops with ADC_ERROR_OVERRUN.
 * On error destination may be partially written; *count is set to zero.
 * After a successful copy the application owns that copy and can send it to
 * CDC independently of DMA. This function does not allocate a transmit queue.
 */
ADC_Status_t ADC_DMA_CopyBlock(uint16_t *destination, uint16_t capacity,
                              uint16_t *count, uint32_t *first_sample_index);

uint8_t ADC_DMA_IsRunning(void);
ADC_Status_t ADC_DMA_GetStatus(void); /* Latched DMA error, otherwise ADC_OK. */
uint32_t ADC_DMA_GetCapturedSamples(void); /* Completed halves, NOT live CNDTR. */
void ADC_DMA_IRQHandler(void);

#ifdef __cplusplus
}
#endif
#endif
