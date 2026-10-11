 #include "ADC.h"

#if !defined(STM32F103xB)
#error "This module targets STM32F103C8/CB. Select STM32F103xB in the project."
#endif

#if ADC_SAMPLE_TIME_BITS > 7U
#error "ADC_SAMPLE_TIME_BITS must be in the range 0..7."
#endif

#if ADC_TIMEOUT_LOOPS == 0U
#error "ADC_TIMEOUT_LOOPS must be greater than zero."
#endif

static uint8_t adc_initialized = 0U;
static uint8_t adc_default_channel = ADC_CH_PA0;

/* A conservative delay of at least 10 us, provided SystemCoreClock is correct.
 * Each iteration contains a NOP; loop overhead only increases the delay.
 */
static void ADC_StabilizationDelay(void)
{
    uint32_t loops = ((SystemCoreClock / 1000000U) + 1U) * 10U;

    while (loops > 0U)
    {
        __NOP();
        --loops;
    }
}

static ADC_Status_t ADC_WaitFlag(volatile uint32_t *reg,
                               uint32_t mask,
                               uint8_t wait_for_set)
{
    uint32_t remaining = ADC_TIMEOUT_LOOPS;

    while (((*reg & mask) != 0U) != (wait_for_set != 0U))
    {
        if (remaining == 0U)
        {
            return ADC_ERROR_TIMEOUT;
        }
        --remaining;
        __NOP();
    }

    return ADC_OK;
}

static void ADC_ConfigureChannel(uint8_t channel)
{
    uint32_t shift = (uint32_t)channel * 3U;

    if (channel <= ADC_CH_PA7)
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
        (void)RCC->APB2ENR;
        /* MODE=00 and CNF=00: analog input. Preserve the other pins. */
        GPIOA->CRL &= ~(0xFUL << ((uint32_t)channel * 4U));
    }
    else
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
        (void)RCC->APB2ENR;
        GPIOB->CRL &= ~(0xFUL << (((uint32_t)channel - 8U) * 4U));
    }

    /* Channels 0..9 have their sample-time fields in SMPR2. */
    ADC1->SMPR2 = (ADC1->SMPR2 & ~(0x7UL << shift)) |
                  ((uint32_t)ADC_SAMPLE_TIME_BITS << shift);

    /* The regular sequence has one conversion, at rank 1. */
    ADC1->SQR3 = (ADC1->SQR3 & ~ADC_SQR3_SQ1) | (uint32_t)channel;
}

static ADC_Status_t ADC_AbortTimeout(void)
{
    ADC1->CR2 &= ~(ADC_CR2_ADON | ADC_CR2_SWSTART);
    adc_initialized = 0U;
    return ADC_ERROR_TIMEOUT;
}

ADC_Status_t ADC_Init(uint8_t channel)
{
    uint32_t ppre2;
    uint32_t pclk2;
    uint32_t adc_clock;

    if (channel > ADC_CH_PB1)
    {
        return ADC_ERROR_PARAM;
    }

    /* CMSIS function: reads RCC registers; does not configure the clock. */
    SystemCoreClockUpdate();
    pclk2 = SystemCoreClock;
    ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11U;
    if (ppre2 >= 4U)
    {
        pclk2 >>= (ppre2 - 3U); /* PPRE2: /2, /4, /8 or /16. */
    }

    adc_clock = pclk2 / 6U;
    if ((adc_clock < 600000U) || (adc_clock > 14000000U))
    {
        return ADC_ERROR_CLOCK;
    }

    adc_initialized = 0U;

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR; /* Ensure the peripheral clock enable has taken effect. */

    /* Disable ADC1 before changing its clock divider. */
    ADC1->CR2 &= ~ADC_CR2_ADON;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;

    RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;

    /* Independent mode, no scan, interrupt, DMA or continuous conversion. */
    ADC1->CR1 = 0U;
    /* EXTSEL=111 selects SWSTART; EXTTRIG enables this trigger on STM32F1.
     * ALIGN=0 selects right alignment. */
    ADC1->CR2 = ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG;
    ADC1->SQR1 = 0U; /* L=0: one conversion. */
    ADC1->SQR2 = 0U;
    ADC1->SQR3 = 0U;
    ADC_ConfigureChannel(channel);

    ADC_StabilizationDelay();
    ADC1->CR2 |= ADC_CR2_ADON;
    /* Covers power-up stabilization and two ADC clocks before calibration. */
    ADC_StabilizationDelay();

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    if (ADC_WaitFlag(&ADC1->CR2, ADC_CR2_RSTCAL, 0U) != ADC_OK)
    {
        return ADC_AbortTimeout();
    }

    ADC1->CR2 |= ADC_CR2_CAL;
    if (ADC_WaitFlag(&ADC1->CR2, ADC_CR2_CAL, 0U) != ADC_OK)
    {
        return ADC_AbortTimeout();
    }

    (void)ADC1->DR; /* Discard the calibration result. */
    ADC1->SR = 0U;
    adc_default_channel = channel;
    adc_initialized = 1U;

    return ADC_OK;
}

ADC_Status_t ADC_Read(uint16_t *value)
{
    return ADC_ReadChannel(adc_default_channel, value);
}

ADC_Status_t ADC_ReadChannel(uint8_t channel, uint16_t *value)
{
    if ((channel > ADC_CH_PB1) || (value == (uint16_t *)0))
    {
        return ADC_ERROR_PARAM;
    }

    if ((adc_initialized == 0U) || ((ADC1->CR2 & ADC_CR2_ADON) == 0U))
    {
        return ADC_ERROR_NOT_INITIALIZED;
    }

    ADC_ConfigureChannel(channel);

    /* Discard old data and clear stale status before starting a new sample. */
    (void)ADC1->DR;
    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    if (ADC_WaitFlag(&ADC1->SR, ADC_SR_EOC, 1U) != ADC_OK)
    {
        return ADC_AbortTimeout();
    }

    /* Reading DR also clears EOC on STM32F1. */
    *value = (uint16_t)(ADC1->DR & ADC_MAX_VALUE);
    return ADC_OK;
}
