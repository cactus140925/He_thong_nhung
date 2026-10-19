#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mcu_mock.h"
#include "usb_cdc_app.h"
#include "usbd_cdc_if.h"
#include "stm32f1xx_it.h"
#define main firmware_entry
#include "../Core/Src/main.c"
#undef main
#include "uart_model.h"

USART_TypeDef mock_usart;
GPIO_TypeDef mock_gpio;
RCC_TypeDef mock_rcc;
AFIO_TypeDef mock_afio;
TIM_TypeDef mock_tim;
DMA_TypeDef mock_dma;
MockDMAChannel mock_dma4, mock_dma5;
DWT_Type mock_dwt;
CoreDebug_Type mock_core_debug;
uint32_t mock_primask, mock_irq_enabled;
uint32_t SystemCoreClock = 72000000U;
USBD_HandleTypeDef hUsbDeviceFS;
PCD_HandleTypeDef hpcd_USB_FS;
extern volatile uint32_t demo_usb_irq_count, demo_uart_irq_count;
static USBD_CDC_HandleTypeDef usb_handle;
static uint32_t tick_ms, captured, phase, sample_rate;
static uint16_t dma_length;
static uint8_t running, force_usb_result, usb_pump_enabled = 1U;
static ADC_DMA_Mode_t mock_dma_mode;
static uint32_t stream_next, monitor_reads;
static uint8_t stream_force_ready;
static char wire[4000000];
static size_t wire_length;
static uint32_t rx_rearms, data_packets, multi_data_packets;

void SystemCoreClockUpdate(void) { }
HAL_StatusTypeDef HAL_Init(void) { return HAL_OK; }
uint32_t HAL_GetTick(void) { return tick_ms; }
void HAL_IncTick(void) { ++tick_ms; }
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *handle) { assert(handle == &hpcd_USB_FS); }
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *cfg) { (void)cfg; return HAL_OK; }
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *cfg, uint32_t latency)
{ (void)cfg; (void)latency; return HAL_OK; }
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *cfg)
{ (void)cfg; return HAL_OK; }
void MX_GPIO_Init(void) { }
void MX_USB_DEVICE_Init(void) { }
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *device, uint8_t *buffer, uint16_t length)
{
    USBD_CDC_HandleTypeDef *handle = device->pClassData;
    handle->TxBuffer = buffer; handle->TxLength = length;
    return USBD_OK;
}
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *device, uint8_t *buffer)
{ ((USBD_CDC_HandleTypeDef *)device->pClassData)->RxBuffer = buffer; return USBD_OK; }
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *device)
{ (void)device; ++rx_rearms; return USBD_OK; }
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *device)
{
    USBD_CDC_HandleTypeDef *handle = device->pClassData;
    if (force_usb_result != USBD_OK) return force_usb_result;
    assert(handle->TxState == 0U);
    handle->TxState = 1U; return USBD_OK;
}
ADC_Status_t ADC_Init(uint8_t channel) { assert(channel == ADC_CH_PA0); return ADC_OK; }
ADC_Status_t ADC_Read(uint16_t *value) { ++monitor_reads; *value = 2048U; return ADC_OK; }
void ADC_StopDMA(void) { running = 0U; phase = 0U; }
ADC_Status_t ADC_StartDMA(uint8_t channel, uint16_t *buffer, uint16_t length, ADC_DMA_Mode_t mode)
{
    uint16_t i;
    assert(channel == ADC_CH_PA0 && length <= 512U && (length & 1U) == 0U);
    mock_dma_mode = mode; stream_next = 0U;
    dma_length = length;
    for (i = 0U; i < length; ++i) buffer[i] = (uint16_t)((i * 29U) & 4095U);
    running = 1U; captured = (mode == ADC_DMA_NORMAL) ? length / 2U : 0U; phase = 1U;
    return ADC_OK;
}
ADC_Status_t ADC_DMA_CopyBlock(uint16_t *destination, uint16_t capacity, uint16_t *count, uint32_t *index)
{
    assert(capacity >= dma_length / 2U);
    *count = 0U;
    if (mock_dma_mode == ADC_DMA_CIRCULAR && running) {
        uint16_t i;
        if (!stream_force_ready && (uint64_t)(tick_ms - capture_started) * sample_rate <
            (uint64_t)(stream_next + dma_length / 2U) * 1000U) return ADC_NO_DATA;
        *index = stream_next; *count = dma_length / 2U;
        for (i = 0U; i < *count; ++i) destination[i] = (uint16_t)(((stream_next + i) * 29U) & 4095U);
        stream_next += *count; captured = stream_next;
        return ADC_OK;
    }
    if (phase == 1U || phase == 3U) {
        *index = phase == 1U ? 0U : dma_length / 2U;
        memcpy(destination, &adc_dma_buffer[*index], (size_t)dma_length / 2U * sizeof(uint16_t));
        *count = dma_length / 2U; ++phase; return ADC_OK;
    }
    if (phase == 2U) {
        /* TC between CopyBlock(NO_DATA) and main's IsRunning check. */
        running = 0U; captured = dma_length; phase = 3U;
    }
    return ADC_NO_DATA;
}
uint8_t ADC_DMA_IsRunning(void) { return running; }
uint32_t ADC_DMA_GetCapturedSamples(void) { return captured; }
Timer_Status_t Timer_Init(uint32_t rate)
{
    sample_rate = rate;
    mock_tim.PSC = 1U; mock_tim.ARR = 36000000U / rate - 1U;
    return TIMER_OK;
}
Timer_Status_t Timer_Start(void) { return TIMER_OK; }
void Timer_Stop(void) { }
void Timer_SetDebugFreeze(uint8_t enabled) { (void)enabled; }
uint32_t Timer_GetClockHz(void) { return 72000000U; }
uint64_t Timer_GetActualFrequencyMilliHz(void) { return (uint64_t)sample_rate * 1000U; }

static void configure_usb(void)
{
    if (hUsbDeviceFS.pClassData != NULL) USBD_Interface_fops_FS.DeInit();
    memset(&usb_handle, 0, sizeof(usb_handle));
    hUsbDeviceFS.pClassData = &usb_handle;
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    USBD_Interface_fops_FS.Init();
    USB_CDC_ClearStatus();
    App_ProcessLinkEvents();
    if (demo_state == APP_DRAIN || demo_state == APP_ABORT_DRAIN) App_ProcessTx();
}
static void inject(const char *text)
{
    size_t length = strlen(text), offset;
    for (offset = 0U; offset < length; offset += 7U) {
        uint32_t part = (uint32_t)((length - offset > 7U) ? 7U : length - offset);
        USBD_Interface_fops_FS.Receive((uint8_t *)(text + offset), &part);
    }
}
static void pump_usb(void)
{
    if (usb_handle.TxState != 0U) {
        assert(usb_handle.TxLength <= USB_CDC_TX_SLOT_BYTES);
        if (usb_handle.TxLength >= 4U && memcmp(usb_handle.TxBuffer, "\xA5\x5A\xC3\x3C", 4U) == 0) {
            uint16_t count = (uint16_t)(usb_handle.TxBuffer[6] | ((uint16_t)usb_handle.TxBuffer[7] << 8U));
            ++data_packets;
            assert(usb_handle.TxLength == APP_BINARY_FRAME_BYTES && count > 0U && count <= ((uint8_t)usb_handle.TxBuffer[5] == 3U ? APP_RNG_PACKED_SAMPLES : APP_BINARY_SAMPLES));
            if (count > 1U) ++multi_data_packets;
        }
        assert(wire_length + usb_handle.TxLength < sizeof(wire));
        memcpy(wire + wire_length, usb_handle.TxBuffer, usb_handle.TxLength);
        wire_length += usb_handle.TxLength;
        usb_handle.TxState = 0U;
    }
}
static void step(uint32_t iteration)
{
    USB_CDC_App_Task();
    App_ProcessLinkEvents();
    App_ProcessCommands();
    if (demo_state != APP_IDLE && demo_state != APP_ERROR && demo_state != APP_ABORT_DRAIN &&
        (!USB_CDC_IsConfigured() || USB_CDC_GetStatus() != USB_CDC_OK)) App_Fail(11U);
    App_ProcessCapture(); App_ProcessUARTTest(); App_ProcessReply(); App_ProcessTx();
    if (usb_pump_enabled && (iteration % 7U) == 0U) pump_usb();
}
static void drain_control(void)
{
    uint32_t i;
    for (i = 0U; i < 1000U && (reply_length || !USB_CDC_IsTxIdle()); ++i) step(i);
    assert(!reply_length && USB_CDC_IsTxIdle());
}
static void inject_uart(const char *text)
{
    model_uart_receive((const uint8_t *)text, (uint32_t)strlen(text), 1U);
}
static void step_uart(void)
{
    uint8_t bytes[12];
    uint32_t count;
    static uint32_t baud_fraction;
    ++tick_ms;
    USB_CDC_App_Task(); App_ProcessLinkEvents(); App_ProcessCommands();
    App_ProcessCapture(); App_ProcessUARTTest(); App_ProcessReply(); App_ProcessTx();
    baud_fraction += 1152U; /* 11.52 payload bytes/ms at 115200, 8N1. */
    count = model_uart_transmit(bytes, baud_fraction / 100U);
    baud_fraction %= 100U;
    assert(wire_length + count < sizeof(wire));
    memcpy(wire + wire_length, bytes, count); wire_length += count;
    /* USB is a physically separate wire: retire it without splicing its
     * BUSY reply into a UART binary packet under test. */
    usb_handle.TxState = 0U;
}
static void drain_uart_control(void)
{
    uint32_t i;
    for (i = 0U; i < 1000U && (reply_length || !UART1_IsTxIdle()); ++i) step_uart();
    assert(!reply_length && UART1_IsTxIdle());
}
static void save_wire(const char *path)
{
    FILE *file = fopen(path, "wb"); assert(file);
    assert(fwrite(wire, 1U, wire_length, file) == wire_length); fclose(file);
}
int main(int argc, char **argv)
{
    uint8_t source[64], line_coding[7], line_new[7] = {0x80U,0x25U,0U,0U,0U,0U,8U};
    uint8_t output[256], *active;
    uint32_t i, generation, old_session;
    char oversized[120];
    mock_rcc.CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADCPRE_DIV6;
    /* Exercise actual project vectors, including an accidental early UART IRQ. */
    USART1_IRQHandler(); assert(demo_uart_irq_count == 1U && mock_irq_enabled == 0U);
    USB_LP_CAN1_RX0_IRQHandler(); assert(demo_usb_irq_count == 1U);
    USB_CDC_App_Init(); configure_usb(); App_Init();
    assert(demo_boot_stage == 5U);
    assert(demo_uart_status == UART1_OK && (mock_usart.CR1 & USART_CR1_UE));
    assert((mock_dma5.CCR & (DMA_CCR_EN | DMA_CCR_CIRC)) == (DMA_CCR_EN | DMA_CCR_CIRC));
    mock_usart.SR = USART_SR_TC;
    assert(App_ADCPeriodIsValid());
    /* Check the new ADC sampling time and external-trigger latency bound.
     * At 12 MHz ADC and 72 MHz PCLK2/TIM3 the bound is 517 timer ticks. */
    assert(ADC_SAMPLE_TIME_BITS == 6U);
    assert(Timer_Init(100000U) == TIMER_OK && App_ADCPeriodIsValid());
    mock_tim.PSC = 0U; mock_tim.ARR = 516U;
    assert(!App_ADCPeriodIsValid());
    mock_tim.ARR = 517U; assert(App_ADCPeriodIsValid());
    assert(Timer_Init(100000U) == TIMER_OK);
    mock_rcc.CFGR |= RCC_CFGR_PPRE2_DIV2;
    assert(!App_ADCPeriodIsValid());
    mock_rcc.CFGR &= ~RCC_CFGR_PPRE2;
    assert(Timer_Init(config.adc_rate) == TIMER_OK && App_ADCPeriodIsValid());
    USBD_Interface_fops_FS.Control(CDC_GET_LINE_CODING, line_coding, 7U);
    assert(line_coding[0] == 0U && line_coding[1] == 0xC2U && line_coding[6] == 8U);
    USBD_Interface_fops_FS.Control(CDC_SET_LINE_CODING, line_new, 7U);
    USBD_Interface_fops_FS.Control(CDC_GET_LINE_CODING, line_coding, 7U);
    assert(memcmp(line_coding, line_new, 7U) == 0);

    /* Exercise the ACTUAL merged transport and branch RX interface. */
    memset(source, 0x55, sizeof(source));
    for (i = 0U; i < USB_CDC_TX_SLOTS; ++i) assert(USB_CDC_TxEnqueue(source, 64U) == USB_CDC_OK);
    assert(USB_CDC_TxEnqueue(source, 1U) == USB_CDC_BUSY);
    memset(source, 0x99, sizeof(source));
    USB_CDC_App_Task(); active = usb_handle.TxBuffer;
    for (i = 0U; i < 64U; ++i) assert(active[i] == 0x55U);
    for (i = 0U; i < 100U; ++i) USB_CDC_App_Task();
    assert(usb_handle.TxBuffer == active && !USB_CDC_IsTxIdle());
    for (i = 0U; i < 4U; ++i) { pump_usb(); USB_CDC_App_Task(); }
    assert(USB_CDC_IsTxIdle());
    force_usb_result = USBD_FAIL;
    assert(USB_CDC_TxEnqueue(source, 3U) == USB_CDC_OK);
    USB_CDC_App_Task(); assert(USB_CDC_GetStatus() == USB_CDC_ERROR_USB);
    force_usb_result = USBD_OK; configure_usb();

    /* Binary DATA and queue BUSY must not advance index or CRC. */
    {
        char staged[64], integer[4];
        uint16_t staged_count, staged_length;
        uint32_t staged_crc;
        App_PutU32(integer, 0x12345678UL);
        assert(memcmp(integer, "\x78\x56\x34\x12", 4U) == 0);
        demo_state = APP_DATA; active_count = demo_received_samples = 6U; demo_sent_samples = 0U;
        crc_state = 0xFFFFFFFFUL;
        for (i = 0U; i < 6U; ++i) captured_samples[i] = (uint16_t)(i * 29U);
        for (i = 0U; i < USB_CDC_TX_SLOTS; ++i) assert(USB_CDC_TxEnqueue(source, 1U) == USB_CDC_OK);
        App_ProcessTx();
        assert(demo_sent_samples == 0U && crc_state == 0xFFFFFFFFUL && data_packet_count > 1U);
        staged_count = data_packet_count; staged_length = data_packet_length; staged_crc = data_packet_crc;
        assert(staged_count == 6U && staged_length == APP_BINARY_FRAME_BYTES);
        assert(memcmp(tx_line, "\xA5\x5A\xC3\x3C\x01\x01\x06\x00", 8U) == 0);
        memcpy(staged, tx_line, staged_length);
        for (i = 0U; i < 20U; ++i) App_ProcessTx();
        assert(demo_sent_samples == 0U && crc_state == 0xFFFFFFFFUL);
        assert(data_packet_count == staged_count && data_packet_crc == staged_crc);
        assert(memcmp(staged, tx_line, staged_length) == 0);
        USB_CDC_App_Task(); pump_usb(); USB_CDC_App_Task();
        App_ProcessTx();
        assert(demo_sent_samples == staged_count && crc_state == staged_crc && data_packet_count == 0U);
        while (!USB_CDC_IsTxIdle()) { USB_CDC_App_Task(); pump_usb(); }
        demo_state = APP_IDLE;
    }

    wire_length = 0U;
    inject("CMD=CONFIG\nADC_RATE=1000\nSAMPLE=512\nSIZE=32\n");
    App_ProcessCommands(); assert(config.sample_count == 512U && reply_length);
    drain_control();
    inject("STA"); App_ProcessCommands(); assert(demo_state == APP_IDLE);
    inject("RT\n"); App_ProcessCommands(); assert(demo_state == APP_CAPTURE);
    App_ProcessCapture(); assert(demo_received_samples == 256U);
    App_ProcessCapture(); assert(!running && demo_received_samples == 256U && demo_state == APP_CAPTURE);
    App_ProcessCapture(); assert(demo_capture_done && demo_received_samples == 512U);
    for (i = 0U; i < 100000U && demo_state != APP_IDLE; ++i) step(i);
    assert(demo_state == APP_IDLE && demo_done && demo_tx_complete && !demo_failed);
    assert(demo_sent_samples == 512U && demo_first_sample_index == 256U && demo_crc32 == 0x0C64BB6BUL);
    assert(multi_data_packets > 0U && data_packets < 40U);
    if (argc > 1) save_wire(argv[1]);

    /* Independent RNG9 mode: same ADC DMA and UART/USB transport, 9-bit words.
     * BEGINR with binary-kind 3 prevents RAW/RNG9 frame confusion. */
    wire_length = 0U;
    inject("CMD=CONFIG\nADC_RATE=1000\nSAMPLE=512\nMODE=RNG9\nSIZE=32\n");
    App_ProcessCommands(); assert(!config_mode && rng9_selected && config.adc_rate == 1000U);
    drain_control();
    inject("START\n"); App_ProcessCommands(); assert(rng9_running && demo_state == APP_CAPTURE);
    for (i = 0U; i < 100000U && demo_state != APP_IDLE; ++i) step(i);
    assert(demo_done && !demo_failed && demo_sent_samples == 512U && rng9_running);
    assert(wire_length > 100U);
    if (argc > 7) save_wire(argv[7]);

    drain_control(); /* ensure final frame retired before next config */
    /* Stateful RNG9 long-stream coverage, 37 packed words/frame. */
    wire_length = 0U; data_packets = 0U;
    inject("CMD=CONFIG\nADC_RATE=3000\nSAMPLE=50000\nMODE=RNG9\nSIZE=32\n");
    App_ProcessCommands(); drain_control();
    assert(!config_mode && rng9_selected && config.adc_rate == 3000U && config.sample_count == 50000U);
    inject("START\n"); App_ProcessCommands();
    for (i = 0U; i < 4000000U && demo_state != APP_IDLE; ++i) {
        if ((i % 64U) == 0U) ++tick_ms;
        step(i);
    }
    assert(demo_done && !demo_failed && demo_sent_samples == 50000U &&
           demo_received_samples == 50000U && rng9_running);
    assert(demo_fifo_peak <= APP_FIFO_SAMPLES && !capture_active);
    assert(data_packets >= (50000U + APP_RNG_PACKED_SAMPLES - 1U) / APP_RNG_PACKED_SAMPLES);
    assert(data_packets <= 1800U); /* DMA boundaries may force short frames. */
    if (argc > 8) save_wire(argv[8]);
    inject("CMD=CONFIG\nADC_RATE=1000\nSAMPLE=512\nMODE=RNG9\nSIZE=32\n");
    App_ProcessCommands(); drain_control();
    assert(rng9_selected && config.adc_rate == 1000U && config.sample_count == 512U);

    /* Ultra-high-speed RNG9 is an opt-in benchmark configuration, not a
     * sustained rate guarantee; ADC DMA/FIFO integrity is checked at runtime. */
    inject("CMD=CONFIG\nADC_RATE=50000\nSAMPLE=512\nMODE=RNG9\nSIZE=32\n");
    App_ProcessCommands();
    assert(!config_mode && rng9_selected && config.adc_rate == 50000U);
    drain_control();

    /* A 50 kHz short RNG9 capture must keep the protocol and CRC intact in
     * this virtual device. This is NOT a 50 kHz MCU throughput guarantee. */
    inject("START\n"); App_ProcessCommands();
    assert(demo_state == APP_CAPTURE && rng9_running);
    for (i = 0U; i < 100000U && demo_state != APP_IDLE; ++i) step(i);
    assert(demo_done && !demo_failed && demo_sent_samples == 512U);
    drain_control();

    /* Switching back to RAW must reset selection and retain old data type. */
    inject("CMD=CONFIG\nADC_RATE=1000\nSAMPLE=512\nSIZE=32\n");
    App_ProcessCommands(); assert(!rng9_selected && config.adc_rate == 1000U);
    drain_control();

    /* Transactional config: invalid values and partial update never commit. */
    inject("CMD=CONFIG\nSAMPLE=513\n"); App_ProcessCommands();
    assert(config.sample_count == 512U && !config_mode && !config_fields); drain_control();
    inject("ADC_RATE=1000junk\n"); App_ProcessCommands();
    assert(!(config_fields & 1U)); drain_control();
    inject("ADC_RATE=4294967296\n"); App_ProcessCommands();
    assert(!(config_fields & 1U)); drain_control();
    inject("CMD=CONFIG\nADC_RATE=1000\nSTART\n"); App_ProcessCommands();
    assert(demo_state == APP_IDLE && config_mode); drain_control();
    wire_length = 0U;
    inject("CMD=CONFIG\nADC_RATE=2000\nSAMPLE=256\nSIZE=128\n");
    App_ProcessCommands(); drain_control();
    inject("START\n"); App_ProcessCommands();
    for (i = 0U; i < 100000U && demo_state != APP_IDLE; ++i) step(i);
    assert(demo_done && !demo_failed && demo_sent_samples == 256U && demo_rate_millihz == 2000000U);
    if (argc > 2) save_wire(argv[2]);

    /* Streaming: bounded MCU RAM, 50,000 samples, one uninterrupted session. */
    wire_length = 0U;
    inject("CMD=CONFIG\nADC_RATE=1000\nSAMPLE=50000\nSIZE=64\n"); App_ProcessCommands(); drain_control();
    assert(config.sample_count == 50000U);
    inject("START\n"); App_ProcessCommands();
    assert(stream_mode && dma_length == 500U && capture_active);
    for (i = 0U; i < 4000000U && demo_state != APP_IDLE; ++i) {
        if ((i % 64U) == 0U) ++tick_ms;
        step(i);
    }
    assert(demo_done && !demo_failed && !capture_active);
    assert(demo_received_samples == 50000U && demo_sent_samples == 50000U);
    if (argc > 3) save_wire(argv[3]);

    /* High requested rates on RAM capture must not be rejected as streaming.
     * This checks state/parameters/CRC, not physical ADC timing or USB speed. */
    {
        static const uint32_t rates[] = {1000U, 10000U, 40000U, 50000U, 75000U, 100000U};
        uint32_t r;
        char settings[80];
        for (r = 0U; r < sizeof(rates) / sizeof(rates[0]); ++r) {
            (void)snprintf(settings, sizeof(settings),
                "CMD=CONFIG\nADC_RATE=%lu\nSAMPLE=512\nSIZE=32\n", (unsigned long)rates[r]);
            inject(settings); App_ProcessCommands(); drain_control();
            assert(config.adc_rate == rates[r] && config.sample_count == 512U);
            inject("START\n"); App_ProcessCommands();
            for (i = 0U; i < 100000U && demo_state != APP_IDLE; ++i) step(i);
            assert(demo_done && !demo_failed && demo_sent_samples == 512U);
            assert(demo_rate_millihz == rates[r] * 1000U && demo_crc32 == 0x0C64BB6BUL);
        }
        inject("CMD=CONFIG\nADC_RATE=1000\nSAMPLE=50000\nSIZE=32\n");
        App_ProcessCommands(); drain_control();
    }

    /* High-rate long captures: state/CRC/packet checks with modeled DMA/USB.
     * 256 foreground turns per virtual ms is a test scheduler, not an MCU
     * benchmark. Physical throughput must be measured on the user's board. */
    {
        static const uint32_t rates[] = {4000U, 8000U, 10000U, 20000U, 30000U, 40000U,
                                         50000U, 60000U, 75000U, 80000U, 100000U};
        uint32_t r;
        char settings[80];
        for (r = 0U; r < sizeof(rates) / sizeof(rates[0]); ++r) {
            wire_length = 0U;
            (void)snprintf(settings, sizeof(settings),
                "CMD=CONFIG\nADC_RATE=%lu\nSAMPLE=50000\nSIZE=32\n", (unsigned long)rates[r]);
            inject(settings); App_ProcessCommands(); drain_control();
            assert(config.adc_rate == rates[r] && config.sample_count == 50000U);
            inject("START\n"); App_ProcessCommands();
            for (i = 0U; i < 4000000U && demo_state != APP_IDLE; ++i) {
                if ((i % 256U) == 0U) ++tick_ms;
                step(i);
            }
            assert(demo_done && !demo_failed && !capture_active && demo_sent_samples == 50000U);
            assert(demo_received_samples == 50000U && demo_crc32 == 0x12B26B2AUL);
            assert(demo_rate_millihz == rates[r] * 1000U);
        }
        /* Longer binary run matching the user's 500k-sample workload. */
        wire_length = 0U;
        inject("CMD=CONFIG\nADC_RATE=100000\nSAMPLE=1000000\nSIZE=32\n");
        App_ProcessCommands(); drain_control();
        inject("START\n"); App_ProcessCommands();
        for (i = 0U; i < 4000000U && demo_state != APP_IDLE; ++i) {
            if ((i % 64U) == 0U) ++tick_ms;
            step(i);
        }
        assert(demo_done && !demo_failed && demo_sent_samples == 1000000U);
        assert(demo_received_samples == 1000000U && !capture_active);
        if (argc > 4) save_wire(argv[4]);
        inject("CMD=CONFIG\nADC_RATE=1000\nSAMPLE=50000\nSIZE=32\n");
        App_ProcessCommands(); drain_control();
    }
    /* Out-of-range requests still leave the previous config intact. */
    inject("CMD=CONFIG\nADC_RATE=100001\nSAMPLE=50000\nSIZE=32\n"); App_ProcessCommands(); drain_control();
    assert(config.adc_rate == 1000U && config.sample_count == 50000U && !config_mode);
    inject("CMD=CONFIG\nADC_RATE=50000\nSAMPLE=65534\nSIZE=32\n"); App_ProcessCommands(); drain_control();
    assert(config.adc_rate == 1000U && config.sample_count == 50000U && !config_mode);
    /* 10 ms of USB backpressure at 100 kS/s fits in the 2048-sample FIFO. */
    inject("CMD=CONFIG\nADC_RATE=100000\nSAMPLE=50000\nSIZE=32\n"); App_ProcessCommands(); drain_control();
    inject("START\n"); App_ProcessCommands();
    usb_pump_enabled = 0U;
    for (i = 0U; i < 640U; ++i) { if ((i % 64U) == 0U) ++tick_ms; step(i); }
    assert(!demo_failed && capture_active && demo_received_samples == 1000U);
    usb_pump_enabled = 1U;
    for (i = 0U; i < 100000U && demo_state != APP_IDLE; ++i) {
        if ((i % 64U) == 0U) ++tick_ms;
        step(i);
    }
    assert(demo_done && !demo_failed && demo_fifo_peak >= 900U);
    /* A slow host must cause an explicit overrun, never skipped samples. */
    inject("START\n"); App_ProcessCommands(); stream_force_ready = 1U;
    for (i = 0U; i < 9U; ++i) App_ProcessCapture();
    assert(demo_failed && demo_error_code == 13U && !capture_active && !running);
    stream_force_ready = 0U;
    for (i = 0U; i < 1000U && demo_state != APP_IDLE; ++i) step(i);
    inject("CMD=CONFIG\nADC_RATE=2000\nSAMPLE=256\nSIZE=32\n"); App_ProcessCommands(); drain_control();

    /* Idle preview uses a real foreground ADC read, not the DMA channel. */
    inject("MONITOR=1\n"); App_ProcessCommands(); drain_control();
    assert(monitor_enabled);
    tick_ms += 50U; App_ProcessMonitor(); drain_control();
    assert(monitor_reads == 1U);
    inject("MONITOR=0\n"); App_ProcessCommands(); drain_control(); assert(!monitor_enabled);

    old_session = demo_session_id;
    memset(oversized, 'X', 100U); memcpy(oversized + 100U, "START\n", 7U);
    inject(oversized);
    for (i = 0U; i < 10U; ++i) App_ProcessCommands();
    assert(demo_state == APP_IDLE && demo_session_id == old_session); drain_control();

    inject("STA"); App_ProcessCommands(); assert(command_length == 3U);
    generation = USB_CDC_GetLinkGeneration(); configure_usb();
    assert(USB_CDC_GetLinkGeneration() != generation && command_length == 0U);
    inject("RT\n"); App_ProcessCommands(); assert(demo_state == APP_IDLE); drain_control();

    memset(output, 'X', sizeof(output));
    { uint32_t n = sizeof(output); USBD_Interface_fops_FS.Receive(output, &n); }
    mock_primask = 1U;
    assert(USB_CDC_RxOverflow() && mock_primask == 1U);
    assert(!USB_CDC_RxOverflow()); mock_primask = 0U;
    configure_usb();
    inject("START\n"); App_ProcessCommands();
    inject("STOP\n"); App_ProcessCommands(); /* START reply still pending: service it first. */
    App_ProcessReply(); App_ProcessCommands();
    assert(demo_failed && demo_error_code == 9U && !running);
    for (i = 0U; i < 1000U && demo_state != APP_IDLE; ++i) step(i);
    assert(demo_state == APP_IDLE && !demo_done);
    {
        size_t offset;
        uint8_t found = 0U;
        for (offset = 0U; offset + 8U <= wire_length; ++offset)
            if (memcmp(wire + offset, "STOP_OK\n", 8U) == 0) { found = 1U; break; }
        assert(found); /* Binary payload contains NUL, so strstr is invalid. */
    }
    inject("START\n"); App_ProcessCommands();
    phase = 0U; tick_ms = capture_started + capture_timeout + 1U;
    App_ProcessCapture(); assert(demo_error_code == 5U && !running);
    configure_usb(); demo_state = APP_IDLE; reply_length = 0U;
    inject("START\n"); App_ProcessCommands(); App_ProcessReply();
    App_ProcessCapture(); App_ProcessCapture(); App_ProcessCapture();
    assert(demo_state == APP_BEGIN);
    while (USB_CDC_TxEnqueue(source, 64U) == USB_CDC_OK) { }
    USB_CDC_App_Task(); active = usb_handle.TxBuffer;
    App_Reply("ERROR:BUSY\n");
    tick_ms = last_tx_progress + APP_TX_TIMEOUT_MS + 1U;
    App_ProcessReply(); assert(reply_length != 0U);
    App_ProcessTx();
    assert(demo_error_code == 8U && demo_state == APP_ERROR && usb_handle.TxBuffer == active);
    /* The team's UART DMA library is also a complete PC transport. */
    configure_usb(); demo_state = APP_IDLE; reply_length = 0U; active_link = command_link = APP_LINK_USB;
    drain_control(); wire_length = 0U;
    inject("UART_TEST=42\n"); App_ProcessCommands();
    assert(demo_state == APP_UART_TEST);
    {
        uint8_t echo[192];
        assert(model_uart_transmit(echo, sizeof(echo)) == 192U);
        model_uart_receive(echo, sizeof(echo), 1U);
        for (i = 0U; i < 8U; ++i) App_ProcessUARTTest();
        assert(demo_state == APP_IDLE && strstr(reply, "UART_TEST,42,PASS,192,192,0,0"));
    }
    drain_control();
    inject("UART_TEST=43\n"); App_ProcessCommands();
    tick_ms = uart_test_started + APP_UART_TEST_TIMEOUT_MS;
    App_ProcessUARTTest();
    assert(strstr(reply, "UART_TEST,43,TIMEOUT"));
    { uint8_t discard[256]; (void)model_uart_transmit(discard, sizeof(discard)); }
    drain_control();
    assert(UART1_Init(115200U) == UART1_OK);
    mock_usart.SR = USART_SR_TC;
    inject_uart("CMD=CONFIG\nADC_RATE=3001\n");
    App_ProcessCommands(); assert(!config_mode && strstr(reply, "ERR=UART_RATE_MAX_3000"));
    drain_uart_control();
    for (generation = 0U; generation < 2U; ++generation) {
        uint32_t wanted = generation ? 50000U : 10000U;
        wire_length = 0U;
        inject_uart("GET_INFO\n"); App_ProcessCommands(); drain_uart_control();
        assert(active_link == APP_LINK_UART && strstr(wire, "HZ=3000"));
        inject_uart(generation ? "CMD=CONFIG\nADC_RATE=3000\nSAMPLE=50000\nSIZE=32\n" :
                                 "CMD=CONFIG\nADC_RATE=1000\nSAMPLE=10000\nSIZE=32\n");
        App_ProcessCommands(); drain_uart_control();
        inject_uart("START\n"); App_ProcessCommands();
        assert(capture_active && active_link == APP_LINK_UART);
        App_ProcessReply(); /* Retire START_OK before injecting the other PC. */
        /* An unrelated USB reset must not stop acquisition over UART. */
        configure_usb();
        assert(capture_active && active_link == APP_LINK_UART && !demo_failed);
        command_link = APP_LINK_USB; App_ProcessLine("STOP");
        assert(capture_active && strstr(reply, "ERR=BUSY_OTHER_LINK"));
        App_ProcessReply();
        for (i = 0U; i < 100000U && demo_state != APP_IDLE; ++i) step_uart();
        assert(demo_state == APP_IDLE && demo_done && !demo_failed && demo_sent_samples == wanted);
        assert(demo_fifo_peak < APP_FIFO_SAMPLES && UART1_IsTxIdle());
        if (argc > (int)(5U + generation)) save_wire(argv[5U + generation]);
    }
    wire_length = 0U;
    inject_uart("START\n"); App_ProcessCommands();
    for (i = 0U; i < 150U; ++i) step_uart();
    inject_uart("STOP\n"); App_ProcessCommands();
    for (i = 0U; i < 1000U && demo_state != APP_IDLE; ++i) step_uart();
    assert(demo_state == APP_IDLE && !demo_done && demo_error_code == 9U);
    {
        size_t offset; uint8_t found = 0U;
        for (offset = 0U; offset + 8U <= wire_length; ++offset)
            if (!memcmp(wire + offset, "STOP_OK\n", 8U)) found = 1U;
        assert(found);
    }
    {
        uint8_t lost_command[300];
        old_session = demo_session_id;
        memset(lost_command, 'X', sizeof(lost_command));
        memcpy(lost_command + sizeof(lost_command) - 6U, "START\n", 6U);
        model_uart_receive(lost_command, sizeof(lost_command), 1U);
        App_ProcessCommands();
        assert(UART1_GetStatus() == UART1_ERROR_RX_OVERFLOW && uart_command_length == 0U);
        for (i = 0U; i < 400U; ++i) step_uart();
        assert(demo_session_id == old_session && !capture_active);
    }
    puts("PASS UART integration: CDC loopback PASS/timeout, UART rate cap, 10k/50k ADC captures at modeled baud budget, USB reset isolation, owner lock, STOP and RX overwrite rejection.");
    printf("PASS merged branch: actual headers/RX/TX; 512 and 256 samples; TC race; CRC; config validation; line coding; backpressure; reset; STOP; capture/TX timeout; RX overflow. RX rearms=%lu\n", (unsigned long)rx_rearms);
    return 0;
}
