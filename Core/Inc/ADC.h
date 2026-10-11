#ifndef CMSIS_ADC_MODULE_H_
#define CMSIS_ADC_MODULE_H_

#include <stdint.h>
#include "stm32f1xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* STM32F103C8/CB: ADC1, 12-bit, right aligned, single conversion, polling. */
#define ADC_CH_PA0  0U
#define ADC_CH_PA1  1U
#define ADC_CH_PA2  2U
#define ADC_CH_PA3  3U
#define ADC_CH_PA4  4U
#define ADC_CH_PA5  5U
#define ADC_CH_PA6  6U
#define ADC_CH_PA7  7U
#define ADC_CH_PB0  8U
#define ADC_CH_PB1  9U

#define ADC_MAX_VALUE 4095U

/* Poll-loop budget, NOT a timeout in milliseconds. No SysTick is required. */
#ifndef ADC_TIMEOUT_LOOPS
#define ADC_TIMEOUT_LOOPS 100000UL
#endif

/* SMPR encoding: 0=1.5, 1=7.5, 2=13.5, 3=28.5, 4=41.5,
 *                5=55.5, 6=71.5, 7=239.5 ADC clock cycles.
 * The long default is useful for potentiometers and sensor dividers.
 */
#ifndef ADC_SAMPLE_TIME_BITS
#define ADC_SAMPLE_TIME_BITS 7U
#endif

typedef enum
{
    ADC_OK = 0,
    ADC_ERROR_PARAM,
    ADC_ERROR_TIMEOUT,
    ADC_ERROR_NOT_INITIALIZED,
    ADC_ERROR_CLOCK
} ADC_Status_t;

/* Call AFTER configuring the system clock.
 * Configures the selected pin as analog input and calibrates ADC1.
 * ADCCLK = PCLK2 / 6: 12 MHz when PCLK2 = 72 MHz.
 * ADC1 and the shared ADC clock divider are owned by this module.
 * Do not run a second ADC1 driver or change clocks while using it.
 */
ADC_Status_t ADC_Init(uint8_t channel);

/* Read the channel selected by ADC_Init(). */
ADC_Status_t ADC_Read(uint16_t *value);

/* Read another channel (0..9), configuring its GPIO as analog input.
 * Does not change the default channel used by ADC_Read().
 * On success: *value = 0..4095. On error: *value is unchanged.
 * On timeout: ADC1 is disabled; call ADC_Init() before trying again.
 * Call these blocking functions sequentially, not from concurrent contexts.
 */
ADC_Status_t ADC_ReadChannel(uint8_t channel, uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif /* CMSIS_ADC_MODULE_H_ */