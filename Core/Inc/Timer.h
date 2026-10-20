#ifndef CMSIS_TIMER_MODULE_H_
#define CMSIS_TIMER_MODULE_H_

#include <stdint.h>
#include "stm32f1xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* STM32F103C8/CB: TIM3 update -> TRGO, internal connection to ADC1.
 * No GPIO, Timer interrupt, Timer DMA request, HAL or LL driver is used.
 * Owns all of TIM3. Public functions are for one foreground context.
 */
typedef enum
{
    TIMER_OK = 0,
    TIMER_ERROR_PARAM,
    TIMER_ERROR_NOT_INITIALIZED,
    TIMER_ERROR_BUSY,
    TIMER_ERROR_CLOCK
} Timer_Status_t;

/* Call after SystemClock_Config(), before ADC_StartDMA().
 * Configures PSC/ARR and update TRGO; leaves TIM3 stopped.
 * frequency_hz: requested integer frequency, >=1 and <=TIM3 clock/2.
 * ARR is kept >=1; STM32 counters are blocked when ARR=0.
 * PSC/ARR rounding means the actual frequency may differ slightly.
 * This is a Timer limit, NOT an ADC conversion-rate limit.
 * Reconfigure only after stopping both Timer and ADC/DMA.
 * Keep system/APB clocks unchanged during acquisition.
 */
Timer_Status_t Timer_Init(uint32_t frequency_hz);

/* Call ONLY after ADC_StartDMA() returns ADC_OK.
 * Starts a fresh full period, without intentionally emitting a startup TRGO.
 * Calling while already running returns TIMER_ERROR_BUSY.
 */
Timer_Status_t Timer_Start(void);

/* Stops future Timer triggers. Does not stop ADC/DMA or discard ADC blocks.
 * An ADC conversion already triggered can still finish.
 * To cancel acquisition: Timer_Stop(), then ADC_StopDMA().
 */
void Timer_Stop(void);
uint8_t Timer_IsRunning(void);

/* Configuration snapshot; 0 before successful Timer_Init().
 * MilliHz uses uint64_t so even high Timer frequencies cannot overflow.
 * Example: 1000000 mHz = 1000 Hz. Fraction below 1 mHz is truncated.
 */
uint32_t Timer_GetClockHz(void);
uint64_t Timer_GetActualFrequencyMilliHz(void);

/* enable!=0: freeze TIM3 when the debugger halts the CPU.
 * Does not freeze an ADC conversion/DMA transfer already in progress.
 * Does not change behavior while the CPU is running.
 */
void Timer_SetDebugFreeze(uint8_t enable);

#ifdef __cplusplus
}
#endif
#endif
