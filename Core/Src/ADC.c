#include "ADC.h"

#if !defined(STM32F103xB)
#error "Select STM32F103xB for STM32F103C8/CB."
#endif
#if ADC_SAMPLE_TIME_BITS > 7U
#error "ADC_SAMPLE_TIME_BITS must be 0..7."
#endif
#if ADC_TIMEOUT_LOOPS == 0U
#error "ADC_TIMEOUT_LOOPS must be greater than zero."
#endif
#if ADC_DMA_IRQ_PRIORITY >= (1U << __NVIC_PRIO_BITS)
#error "ADC_DMA_IRQ_PRIORITY is outside the implemented NVIC priority range."
#endif

#define ADC_DMA_FLAGS (DMA_ISR_HTIF1 | DMA_ISR_TCIF1 | DMA_ISR_TEIF1)
#define ADC_DMA_EVENTS (DMA_ISR_HTIF1 | DMA_ISR_TCIF1)
#define ADC_SOFTWARE_TRIGGER (ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG)

static uint8_t adc_initialized;
static uint8_t adc_default_channel = ADC_CH_PA0;
static volatile uint8_t dma_running;
static volatile uint8_t dma_pending; /* Bit 0: first half; bit 1: second half. */
static volatile ADC_Status_t dma_status = ADC_OK;
static ADC_DMA_Mode_t dma_mode;
static volatile uint16_t *dma_buffer;
static uint16_t dma_length;
static volatile uint32_t dma_index[2];
static volatile uint32_t dma_captured;

static uint32_t ADC_Lock(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return previous;
}

static void ADC_Unlock(uint32_t previous)
{
    __DMB();
    __set_PRIMASK(previous);
}

static void ADC_StabilizationDelay(void)
{
    uint32_t loops = ((SystemCoreClock / 1000000U) + 1U) * 10U;
    while (loops > 0U)
    {
        __NOP();
        --loops;
    }
}

static ADC_Status_t ADC_WaitFlag(volatile uint32_t *reg, uint32_t mask,
                               uint8_t wait_for_set)
{
    uint32_t remaining = ADC_TIMEOUT_LOOPS;
    while (((*reg & mask) != 0U) != (wait_for_set != 0U))
    {
        if (remaining == 0U) return ADC_ERROR_TIMEOUT;
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
        GPIOA->CRL &= ~(0xFUL << ((uint32_t)channel * 4U));
    }
    else
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
        (void)RCC->APB2ENR;
        GPIOB->CRL &= ~(0xFUL << (((uint32_t)channel - 8U) * 4U));
    }
    ADC1->SMPR2 = (ADC1->SMPR2 & ~(0x7UL << shift)) |
                  ((uint32_t)ADC_SAMPLE_TIME_BITS << shift);
    ADC1->SQR3 = (ADC1->SQR3 & ~ADC_SQR3_SQ1) | (uint32_t)channel;
}

static void ADC_Enable(void)
{
    if ((ADC1->CR2 & ADC_CR2_ADON) == 0U)
    {
        ADC1->CR2 |= ADC_CR2_ADON;
        ADC_StabilizationDelay();
    }
}

/* Stop incoming conversions before disabling DMA. No Timer registers touched.
 * Writing ADON=0 stops ADC; calibration/settings are retained.
 */
static void ADC_DMA_DisableHardware(void)
{
    ADC1->CR2 = ADC_SOFTWARE_TRIGGER; /* ADON=0, DMA=0, SWSTART=0. */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    dma_running = 0U;
}

static void ADC_DMA_Fail(ADC_Status_t error)
{
    ADC_DMA_DisableHardware();
    dma_pending = 0U; /* Never return possibly overwritten data. */
    dma_status = error;
}

static ADC_Status_t ADC_AbortTimeout(void)
{
    ADC1->CR2 = ADC_SOFTWARE_TRIGGER;
    adc_initialized = 0U;
    return ADC_ERROR_TIMEOUT;
}

ADC_Status_t ADC_Init(uint8_t channel)
{
    uint32_t ppre2;
    uint32_t pclk2;
    uint32_t adc_clock;

    if (channel > ADC_CH_PB1) return ADC_ERROR_PARAM;
    if ((dma_running != 0U) || (dma_pending != 0U)) return ADC_ERROR_BUSY;

    SystemCoreClockUpdate(); /* CMSIS; reads the current clock configuration. */
    pclk2 = SystemCoreClock;
    ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11U;
    if (ppre2 >= 4U) pclk2 >>= (ppre2 - 3U);
    adc_clock = pclk2 / 6U;
    if ((adc_clock < 600000U) || (adc_clock > 14000000U))
        return ADC_ERROR_CLOCK;

    adc_initialized = 0U;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;
    ADC1->CR2 &= ~ADC_CR2_ADON;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;
    RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;

    ADC1->CR1 = 0U; /* Independent, one channel, no ADC interrupt. */
    ADC1->CR2 = ADC_SOFTWARE_TRIGGER;
    ADC1->SQR1 = 0U; /* L=0: one conversion per trigger. */
    ADC1->SQR2 = 0U;
    ADC1->SQR3 = 0U;
    ADC_ConfigureChannel(channel);
    ADC_StabilizationDelay();
    ADC_Enable();

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    if (ADC_WaitFlag(&ADC1->CR2, ADC_CR2_RSTCAL, 0U) != ADC_OK)
        return ADC_AbortTimeout();
    ADC1->CR2 |= ADC_CR2_CAL;
    if (ADC_WaitFlag(&ADC1->CR2, ADC_CR2_CAL, 0U) != ADC_OK)
        return ADC_AbortTimeout();

    (void)ADC1->DR;
    ADC1->SR = 0U;
    adc_default_channel = channel;
    adc_initialized = 1U;
    dma_status = ADC_OK;
    return ADC_OK;
}

ADC_Status_t ADC_Read(uint16_t *value)
{
    return ADC_ReadChannel(adc_default_channel, value);
}

ADC_Status_t ADC_ReadChannel(uint8_t channel, uint16_t *value)
{
    if ((channel > ADC_CH_PB1) || (value == (uint16_t *)0))
        return ADC_ERROR_PARAM;
    if (adc_initialized == 0U) return ADC_ERROR_NOT_INITIALIZED;
    if ((dma_running != 0U) || (dma_pending != 0U)) return ADC_ERROR_BUSY;

    /* First force ADON=0. Rewriting an unchanged CR2 with ADON=1 on STM32F1
     * can itself launch a conversion, before SWSTART is explicitly requested. */
    ADC1->CR2 = ADC_SOFTWARE_TRIGGER;
    ADC_ConfigureChannel(channel);
    ADC_Enable();
    (void)ADC1->DR;
    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    if (ADC_WaitFlag(&ADC1->SR, ADC_SR_EOC, 1U) != ADC_OK)
        return ADC_AbortTimeout();
    *value = (uint16_t)(ADC1->DR & ADC_MAX_VALUE);
    return ADC_OK;
}

ADC_Status_t ADC_StartDMA(uint8_t channel, uint16_t *buffer,
                          uint16_t length, ADC_DMA_Mode_t mode)
{
    uint32_t previous;
    if ((channel > ADC_CH_PB1) || (buffer == (uint16_t *)0) ||
        (((uintptr_t)buffer & 1U) != 0U) || (length < 2U) ||
        ((length & 1U) != 0U) ||
        ((mode != ADC_DMA_NORMAL) && (mode != ADC_DMA_CIRCULAR)))
        return ADC_ERROR_PARAM;
    if (adc_initialized == 0U) return ADC_ERROR_NOT_INITIALIZED;
    if ((dma_running != 0U) || (dma_pending != 0U)) return ADC_ERROR_BUSY;

    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    (void)RCC->AHBENR;
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    DMA1_Channel1->CCR = 0U;
    DMA1->IFCR = DMA_IFCR_CGIF1; /* Write-one-to-clear for this channel only. */
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);

    ADC1->CR2 = ADC_SOFTWARE_TRIGGER;
    ADC_ConfigureChannel(channel);
    ADC_Enable();
    (void)ADC1->DR;
    ADC1->SR = 0U;

    DMA1_Channel1->CPAR = (uint32_t)(uintptr_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)(uintptr_t)buffer;
    DMA1_Channel1->CNDTR = length;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
                        DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE |
                        DMA_CCR_PL_1; /* High priority, peripheral -> memory. */
    if (mode == ADC_DMA_CIRCULAR) DMA1_Channel1->CCR |= DMA_CCR_CIRC;

    previous = ADC_Lock();
    dma_buffer = buffer;
    dma_length = length;
    dma_mode = mode;
    dma_pending = 0U;
    dma_status = ADC_OK;
    dma_captured = 0U;
    dma_index[0] = 0U;
    dma_index[1] = 0U;
    dma_running = 1U;
    NVIC_SetPriority(DMA1_Channel1_IRQn, ADC_DMA_IRQ_PRIORITY);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    /* EXTSEL=100: TIM3 TRGO. CONT=0: one sample for every Timer trigger.
     * DMA is armed before enabling the external ADC trigger. */
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2;
    ADC_Unlock(previous);
    return ADC_OK;
}

void ADC_StopDMA(void)
{
    uint32_t previous = ADC_Lock();
    /* Avoid touching a DMA peripheral whose clock has never been enabled. */
    if ((RCC->AHBENR & RCC_AHBENR_DMA1EN) != 0U)
    {
        ADC_DMA_DisableHardware();
        DMA1->IFCR = DMA_IFCR_CGIF1;
        NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    }
    dma_running = 0U;
    dma_pending = 0U;
    ADC_Unlock(previous);
}

void ADC_DMA_IRQHandler(void)
{
    uint32_t flags = DMA1->ISR & ADC_DMA_FLAGS;
    if (flags == 0U) return;
    /* Flag positions in ISR and IFCR correspond for HT, TC and TE. */
    DMA1->IFCR = flags;
    if (dma_running == 0U) return;
    if ((flags & DMA_ISR_TEIF1) != 0U)
    {
        ADC_DMA_Fail(ADC_ERROR_DMA);
        return;
    }

    if (dma_mode == ADC_DMA_CIRCULAR)
    {
        /* Coalesced HT+TC or an unconsumed previous block means its valid
         * processing window has ended. Stop rather than return stale data. */
        if (((flags & ADC_DMA_EVENTS) == ADC_DMA_EVENTS) || (dma_pending != 0U))
        {
            ADC_DMA_Fail(ADC_ERROR_OVERRUN);
            return;
        }
    }

    if ((flags & DMA_ISR_HTIF1) != 0U)
    {
        dma_index[0] = dma_captured;
        dma_captured += dma_length / 2U;
        __DMB();
        dma_pending |= 1U;
    }
    if ((flags & DMA_ISR_TCIF1) != 0U)
    {
        dma_index[1] = dma_captured;
        dma_captured += dma_length / 2U;
        __DMB();
        dma_pending |= 2U;
        if (dma_mode == ADC_DMA_NORMAL) ADC_DMA_DisableHardware();
    }
}

/* For circular mode, a completed half must not be the half now being written.
 * Pending boundary/error flags also reveal a deadline missed while IRQs were
 * masked. Merely disabling CPU interrupts does NOT pause DMA transfers.
 */
static ADC_Status_t ADC_DMA_CheckWindow(uint8_t half)
{
    uint32_t flags = DMA1->ISR & ADC_DMA_FLAGS;
    uint32_t remaining = DMA1_Channel1->CNDTR;
    uint8_t writing_half;
    if ((flags & DMA_ISR_TEIF1) != 0U) return ADC_ERROR_DMA;
    if (dma_mode == ADC_DMA_NORMAL) return ADC_OK;
    if ((flags & ADC_DMA_EVENTS) != 0U) return ADC_ERROR_OVERRUN;
    writing_half = ((remaining > 0U) && (remaining <= dma_length / 2U)) ? 1U : 0U;
    return (writing_half == half) ? ADC_ERROR_OVERRUN : ADC_OK;
}

ADC_Status_t ADC_DMA_CopyBlock(uint16_t *destination, uint16_t capacity,
                              uint16_t *count, uint32_t *first_sample_index)
{
    uint32_t previous;
    uint32_t index;
    uint16_t half_length;
    uint16_t offset;
    uint16_t i;
    uint8_t half;
    uint8_t bit;
    uintptr_t dst;
    uintptr_t src;
    ADC_Status_t status;

    if (count != (uint16_t *)0) *count = 0U;
    if ((destination == (uint16_t *)0) || (count == (uint16_t *)0) ||
        (first_sample_index == (uint32_t *)0) ||
        (((uintptr_t)destination & 1U) != 0U)) return ADC_ERROR_PARAM;
    previous = ADC_Lock();
    if (dma_status != ADC_OK)
    {
        status = dma_status;
        ADC_Unlock(previous);
        return status;
    }
    if (dma_pending == 0U)
    {
        ADC_Unlock(previous);
        return ADC_NO_DATA;
    }
    half_length = dma_length / 2U;
    dst = (uintptr_t)destination;
    src = (uintptr_t)dma_buffer;
    if ((capacity < half_length) ||
        ((dst < src + (uintptr_t)dma_length * sizeof(uint16_t)) &&
         (dst + (uintptr_t)half_length * sizeof(uint16_t) > src)))
    {
        ADC_Unlock(previous);
        return ADC_ERROR_PARAM;
    }
    half = ((dma_pending & 1U) != 0U) ? 0U : 1U;
    bit = (uint8_t)(1U << half);
    index = dma_index[half];
    status = ADC_DMA_CheckWindow(half);
    if (status != ADC_OK)
    {
        ADC_DMA_Fail(status);
        ADC_Unlock(previous);
        return status;
    }
    ADC_Unlock(previous);

    offset = (uint16_t)((uint16_t)half * half_length);
    for (i = 0U; i < half_length; ++i)
        destination[i] = dma_buffer[offset + i];

    /* DMA continued during the copy. Validate again before publishing it. */
    previous = ADC_Lock();
    status = dma_status;
    if (status == ADC_OK) status = ADC_DMA_CheckWindow(half);
    if (status != ADC_OK)
    {
        ADC_DMA_Fail(status);
        ADC_Unlock(previous);
        return status;
    }
    dma_pending &= (uint8_t)~bit;
    *first_sample_index = index;
    *count = half_length;
    ADC_Unlock(previous);
    return ADC_OK;
}

uint8_t ADC_DMA_IsRunning(void) { return dma_running; }
ADC_Status_t ADC_DMA_GetStatus(void) { return dma_status; }
uint32_t ADC_DMA_GetCapturedSamples(void) { return dma_captured; }

#if ADC_DMA_DEFINE_IRQHANDLER
void DMA1_Channel1_IRQHandler(void)
{
    ADC_DMA_IRQHandler();
}
#endif
