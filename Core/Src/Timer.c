#include "Timer.h"

#if !defined(STM32F103xB)
#error "Select STM32F103xB for STM32F103C8/CB."
#endif

static uint8_t timer_initialized;
static uint32_t timer_clock_hz;
static uint64_t timer_actual_millihz;

static Timer_Status_t Timer_ReadClock(uint32_t *clock_hz)
{
    uint32_t ppre1;
    uint32_t pclk1;

    SystemCoreClockUpdate(); /* CMSIS reads RCC; no clock configuration changed. */
    if ((SystemCoreClock == 0U) || (SystemCoreClock > 72000000U))
        return TIMER_ERROR_CLOCK;

    ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8U;
    pclk1 = SystemCoreClock;
    if (ppre1 >= 4U) pclk1 >>= (ppre1 - 3U);
    /* STM32F103 APB1 is limited to 36 MHz. */
    if ((pclk1 == 0U) || (pclk1 > 36000000U))
        return TIMER_ERROR_CLOCK;
    *clock_hz = (ppre1 >= 4U) ? (pclk1 * 2U) : pclk1;
    return TIMER_OK;
}

/* TIM3 must be stopped. Reload the internal prescaler and reset the counter.
 * MMS=001 (Enable) makes TRGO follow CEN=0 during UG.
 * MMS=000 (Reset) is NOT a safe "TRGO disabled" mode: UG itself is its TRGO.
 * Return to MMS=010 (Update) before allowing the Timer to run.
 */
static void Timer_ReloadStopped(void)
{
    TIM3->CR2 = TIM_CR2_MMS_0;
    TIM3->CNT = 0U;
    TIM3->EGR = TIM_EGR_UG;
    (void)TIM3->SR;
    TIM3->SR = 0U;
    TIM3->CR2 = TIM_CR2_MMS_1;
}

Timer_Status_t Timer_Init(uint32_t frequency_hz)
{
    uint32_t clock_hz;
    uint32_t total_divider;
    uint32_t psc_divider;
    uint32_t arr_divider;
    Timer_Status_t status;

    if (frequency_hz == 0U) return TIMER_ERROR_PARAM;
    if (Timer_IsRunning() != 0U) return TIMER_ERROR_BUSY;
    status = Timer_ReadClock(&clock_hz);
    if (status != TIMER_OK) return status;
    if (frequency_hz > clock_hz / 2U) return TIMER_ERROR_PARAM;

    total_divider = (uint32_t)(((uint64_t)clock_hz + frequency_hz / 2U) /
                              frequency_hz);
    psc_divider = ((total_divider - 1U) / 65536U) + 1U;
    arr_divider = (total_divider + psc_divider / 2U) / psc_divider;
    if ((psc_divider > 65536U) || (arr_divider < 2U) ||
        (arr_divider > 65536U)) return TIMER_ERROR_PARAM;

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
    TIM3->CR1 = TIM_CR1_ARPE; /* Upcount, internal clock, continuous, CEN=0. */
    TIM3->CR2 = TIM_CR2_MMS_0;
    TIM3->SMCR = 0U;
    TIM3->DIER = 0U; /* No UIE/UDE: ADC requests DMA, not this Timer. */
    TIM3->CCER = 0U;
    TIM3->CCMR1 = 0U;
    TIM3->CCMR2 = 0U;
    TIM3->PSC = psc_divider - 1U;
    TIM3->ARR = arr_divider - 1U;
    Timer_ReloadStopped();

    timer_clock_hz = clock_hz;
    timer_actual_millihz = ((uint64_t)clock_hz * 1000U) /
                          ((uint64_t)psc_divider * arr_divider);
    timer_initialized = 1U;
    return TIMER_OK;
}

Timer_Status_t Timer_Start(void)
{
    uint32_t current_clock;
    Timer_Status_t status;
    if (timer_initialized == 0U) return TIMER_ERROR_NOT_INITIALIZED;
    if (Timer_IsRunning() != 0U) return TIMER_ERROR_BUSY;
    status = Timer_ReadClock(&current_clock);
    if (status != TIMER_OK) return status;
    if (current_clock != timer_clock_hz) return TIMER_ERROR_CLOCK;

    /* ADC DMA is already armed. Suppress UG TRGO while resetting the
     * prescaler, then enable update TRGO before the first real overflow. */
    Timer_ReloadStopped();
    TIM3->CR1 |= TIM_CR1_CEN;
    return TIMER_OK;
}

void Timer_Stop(void)
{
    if (timer_initialized != 0U) TIM3->CR1 &= ~TIM_CR1_CEN;
}

uint8_t Timer_IsRunning(void)
{
    if (timer_initialized == 0U) return 0U;
    return ((TIM3->CR1 & TIM_CR1_CEN) != 0U) ? 1U : 0U;
}

uint32_t Timer_GetClockHz(void) { return timer_clock_hz; }
uint64_t Timer_GetActualFrequencyMilliHz(void) { return timer_actual_millihz; }

void Timer_SetDebugFreeze(uint8_t enable)
{
    if (enable != 0U) DBGMCU->CR |= DBGMCU_CR_DBG_TIM3_STOP;
    else DBGMCU->CR &= ~DBGMCU_CR_DBG_TIM3_STOP;
}
