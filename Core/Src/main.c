/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_cdc_app.h"
#include "ADC.h"
#include "Timer.h"
#include "UART.h"
#include "rng9.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define APP_RAM_SAMPLES 512U
#define APP_DMA_HALF_SAMPLES 250U
#define APP_FIFO_SAMPLES 2048U
#define APP_MAX_SAMPLES 1000000UL
#define APP_MAX_RATE_HZ 200000U
#define APP_STREAM_MAX_RATE APP_MAX_RATE_HZ
#define APP_BLOCK_CAPACITY (APP_RAM_SAMPLES / 2U)
#define APP_TX_TIMEOUT_MS 5000U
#define APP_BINARY_SAMPLES 23U /* RAW, 46 payload bytes */
#define APP_RNG_PACKED_SAMPLES 40U /* 40*9 bits = 45 bytes + 1 pad byte */
#define APP_BINARY_FRAME_BYTES 62U
#define APP_RAW_FAST_SAMPLES 116U /* USB-only v3 RAW frame; UART/RNG9 remain wire v2. */
#define APP_RAW_FAST_FRAME_BYTES 248U
#if APP_RAW_FAST_FRAME_BYTES != (16U + 2U * APP_RAW_FAST_SAMPLES)
#error "RAW v3 frame and payload capacity disagree"
#endif
#if APP_RAW_FAST_FRAME_BYTES > USB_CDC_TX_SLOT_BYTES
#error "USB CDC TX slots are too small for RAW v3"
#endif
#define APP_UART_TEST_BYTES 192U
#define APP_UART_TEST_TIMEOUT_MS 1000U
#define APP_UART_MAX_RATE_HZ 3000U
#define APP_RNG9_MAX_RATE_HZ 10000U /* Long RNG9 runs: board-observed operating ceiling. */
#define APP_RNG9_SLICE_SAMPLES 8U /* Yield to capture/CDC between bounded work slices. */
#define APP_CONFIG_TIMEOUT_MS 5000U
#define APP_POST_BIN_SAMPLES 500U /* POST_BIN block: 1000 LE uint16 bytes, fits the 2048 FIFO. */
#define APP_POST_BIN_READ_BYTES 64U
typedef enum {
    APP_IDLE = 0, APP_CAPTURE, APP_BEGIN, APP_DATA, APP_END, APP_DRAIN,
    APP_ERROR, APP_ABORT_DRAIN, APP_STOP_ACK, APP_METRICS, APP_UART_TEST
} AppState;
typedef struct { uint32_t adc_rate, sample_count; uint16_t random_size; } AppConfig;
typedef enum { APP_LINK_USB = 0, APP_LINK_UART } AppLink;
static AppLink active_link, command_link, reply_link;
static AppConfig config = {50000U, 100000U, 32U};
static AppConfig draft_config;
static uint8_t rng9_selected, rng9_draft, rng9_running;
static RNG9_State rng9_state, rng9_packet_state;
static uint8_t config_mode, config_fields, capture_active, stream_mode, monitor_enabled;
static uint32_t active_count;
static uint16_t dma_block_count;
static uint16_t adc_dma_buffer[APP_RAM_SAMPLES];
static uint16_t adc_block[APP_BLOCK_CAPACITY];
/* RAM capture for short runs; bounded sample FIFO for longer runs. */
static uint16_t captured_samples[APP_FIFO_SAMPLES];
static char command[160], reply[64];
static char tx_line[APP_RAW_FAST_FRAME_BYTES];
static uint16_t command_length, reply_length;
static char uart_command[96];
static uint16_t uart_command_length;
static uint8_t uart_command_overflow;
static uint32_t config_started;
/* Keep a prepared DATA packet across BUSY without repeating formatting.
 * Samples stay owned by the FIFO until this entire packet is accepted. */
static uint16_t data_packet_length, data_packet_count, data_packet_progress;
static uint16_t data_packet_payload_bytes;
static uint32_t data_packet_accumulator, data_packet_work_cycles;
static uint8_t data_packet_bits;
static uint8_t command_overflow, post_processing;
static uint32_t post_rx_tick;
/* Pending POST_BIN payload: nonzero count switches USB RX from lines to bytes. */
static uint32_t post_bin_first;
static uint16_t post_bin_count, post_bin_bytes;
static uint32_t capture_started, capture_timeout, last_tx_progress, monitor_tick;
static uint32_t seen_link_generation;
static uint8_t cycle_counter_available;
static uint32_t capture_elapsed_ms;
static uint8_t uart_test_payload[APP_UART_TEST_BYTES];
static uint32_t uart_test_id, uart_test_started;
static uint16_t uart_test_received, uart_test_mismatches;
volatile AppState demo_state;
volatile uint32_t demo_session_id, demo_received_samples, demo_sent_samples;
volatile uint32_t demo_first_sample_index, demo_rate_millihz, demo_error_code;
volatile uint32_t demo_uart_dropped_logs;
volatile uint32_t demo_boot_stage;
volatile uint32_t demo_fifo_peak, demo_copy_cycles_max, demo_pack_cycles_max;
volatile uint32_t demo_rng_slice_cycles_max;
volatile uint8_t demo_capture_done, demo_tx_complete, demo_done, demo_failed;
volatile ADC_Status_t demo_status;
volatile Timer_Status_t demo_timer_status;
volatile UART1_Status_t demo_uart_status;

static void App_Log(const char *text)
{
    /* USART1 now carries the framed PC protocol. Never interleave raw logs. */
    (void)text;
}
static uint8_t App_LinkConfigured(AppLink link)
{
    return link == APP_LINK_USB ? USB_CDC_IsConfigured() : (uint8_t)((USART1->CR1 & USART_CR1_UE) != 0U);
}
static uint8_t App_LinkTxIdle(AppLink link)
{
    return link == APP_LINK_USB ? USB_CDC_IsTxIdle() : UART1_IsTxIdle();
}
static USB_CDC_Status_t App_LinkWrite(AppLink link, const uint8_t *data, uint16_t length)
{
    UART1_Status_t status;
    if (link == APP_LINK_USB) return USB_CDC_TxEnqueue(data, length);
    status = UART1_Write(data, length);
    return status == UART1_OK ? USB_CDC_OK : status == UART1_BUSY ? USB_CDC_BUSY : USB_CDC_ERROR_USB;
}
static uint8_t App_LinkHealthy(AppLink link)
{
    return link == APP_LINK_USB ? (uint8_t)(USB_CDC_GetStatus() == USB_CDC_OK)
                               : (uint8_t)(UART1_GetStatus() == UART1_OK);
}
static uint32_t App_Cycles(void)
{
    return cycle_counter_available ? DWT->CYCCNT : 0U;
}
static void App_PutU16(char *out, uint16_t value)
{
    out[0] = (char)(uint8_t)value; out[1] = (char)(uint8_t)(value >> 8U);
}
static void App_PutU32(char *out, uint32_t value)
{
    App_PutU16(out, (uint16_t)value); App_PutU16(out + 2, (uint16_t)(value >> 16U));
}
/* Exact wire-compatible bit0-first RNG9 packing without a per-bit loop. */
static uint16_t App_Reverse9(uint16_t word)
{
    static const uint8_t reverse4[16] = {0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15};
    return (uint16_t)(((uint16_t)reverse4[word & 15U] << 5U) |
                      ((uint16_t)reverse4[(word >> 4U) & 15U] << 1U) |
                      ((word >> 8U) & 1U));
}
static uint8_t App_PrepareDataPacket(void)
{
    uint32_t cycles_started = App_Cycles();
    uint16_t i, end;
    if (data_packet_count == 0U) {
        uint32_t available = demo_received_samples - demo_sent_samples;
        uint16_t limit = rng9_running ? APP_RNG_PACKED_SAMPLES :
            (active_link == APP_LINK_USB ? APP_RAW_FAST_SAMPLES : APP_BINARY_SAMPLES);
        data_packet_count = available > limit ? limit : (uint16_t)available;
        data_packet_progress = data_packet_payload_bytes = 0U;
        data_packet_length = data_packet_bits = 0U;
        data_packet_accumulator = data_packet_work_cycles = 0U;
        rng9_packet_state = rng9_state;
        /* v2 UART/RNG9: 16-byte header + 46-byte payload (62 bytes).
         * v3 USB RAW: 16-byte header + 232-byte payload (248 bytes).
         * Keep the frame in app RAM until fully copied into the TX queue. */
        memset(tx_line, 0, sizeof(tx_line));
        memcpy(tx_line, "\xA5\x5A\xC3\x3C", 4U);
        /* Wire version 3 is only for large USB RAW frames.
         * Keep legacy 62-byte wire v2 for RNG9 and UART. */
        tx_line[4] = (!rng9_running && active_link == APP_LINK_USB) ? 3 : 2;
        tx_line[5] = rng9_running ? 3 : 1;
        App_PutU16(tx_line + 6, data_packet_count);
        App_PutU32(tx_line + 8, demo_session_id);
        App_PutU32(tx_line + 12, demo_sent_samples);
    }
    end = data_packet_count;
    if (rng9_running && (uint16_t)(end - data_packet_progress) > APP_RNG9_SLICE_SAMPLES)
        end = (uint16_t)(data_packet_progress + APP_RNG9_SLICE_SAMPLES);
    for (i = data_packet_progress; i < end; ++i) {
        uint16_t value = captured_samples[(demo_sent_samples + i) % APP_FIFO_SAMPLES];
        if (rng9_running) {
            value = RNG9_Next(&rng9_packet_state, value);
            data_packet_accumulator = (data_packet_accumulator << 9U) | App_Reverse9(value);
            data_packet_bits = (uint8_t)(data_packet_bits + 9U);
            while (data_packet_bits >= 8U) {
                data_packet_bits = (uint8_t)(data_packet_bits - 8U);
                tx_line[16U + data_packet_payload_bytes++] =
                    (char)(uint8_t)(data_packet_accumulator >> data_packet_bits);
            }
            data_packet_accumulator &= (1UL << data_packet_bits) - 1UL;
        } else App_PutU16(tx_line + 16U + i * 2U, value);
    }
    data_packet_progress = end;
    if (end == data_packet_count) {
        if (rng9_running && data_packet_bits != 0U)
            tx_line[16U + data_packet_payload_bytes] =
                (char)(uint8_t)(data_packet_accumulator << (8U - data_packet_bits));
        data_packet_length = (!rng9_running && active_link == APP_LINK_USB) ?
            APP_RAW_FAST_FRAME_BYTES : APP_BINARY_FRAME_BYTES;
    }
    if (cycle_counter_available) {
        uint32_t elapsed = App_Cycles() - cycles_started;
        data_packet_work_cycles += elapsed;
        if (rng9_running && elapsed > demo_rng_slice_cycles_max) demo_rng_slice_cycles_max = elapsed;
        if (data_packet_length != 0U && data_packet_work_cycles > demo_pack_cycles_max)
            demo_pack_cycles_max = data_packet_work_cycles;
    }
    return (uint8_t)(data_packet_length != 0U);
}
static uint8_t App_ADCPeriodIsValid(void)
{
    static const uint16_t cycles_x2[8] = {3U, 15U, 27U, 57U, 83U, 111U, 143U, 479U};
    static const uint8_t dividers[4] = {2U, 4U, 6U, 8U};
    uint32_t ppre = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11U;
    uint32_t pclk = SystemCoreClock;
    uint32_t adc_div = dividers[(RCC->CFGR & RCC_CFGR_ADCPRE) >> 14U];
    uint64_t period = ((uint64_t)TIM3->PSC + 1U) * ((uint64_t)TIM3->ARR + 1U);
    /* Datasheet: Ts+12.5 cycles, regular latency 2 ADC cycles,
     * plus 1 PCLK2 cycle for an external trigger. All in half cycles. */
    uint64_t conversion = cycles_x2[ADC_GetSampleTimeBits()] + 25U + 4U;
    if (ppre >= 4U) pclk >>= (ppre - 3U);
    return (uint8_t)(2U * period * pclk > (conversion * adc_div + 2U) * Timer_GetClockHz());
}
static void App_Reply(const char *text)
{
    size_t length = strlen(text);
    if (length < sizeof(reply)) {
        memcpy(reply, text, length); reply_length = (uint16_t)length;
        reply_link = command_link;
    }
}
static void App_Fail(uint32_t code)
{
    Timer_Stop(); ADC_StopDMA(); capture_active = monitor_enabled = 0U;
    demo_failed = 1U; demo_error_code = code; demo_tx_complete = demo_done = 0U;
    data_packet_count = 0U; post_processing = 0U; post_bin_count = 0U;
    reply_length = 0U;
    last_tx_progress = HAL_GetTick(); demo_state = APP_ERROR;
    App_Log("ADC_ERROR\r\n");
}
static uint8_t App_ParseU32(const char *text, uint32_t *value)
{
    uint32_t number = 0U;
    if (*text == '\0') return 0U;
    while (*text != '\0') {
        uint32_t digit;
        if (*text < '0' || *text > '9') return 0U;
        digit = (uint32_t)(*text++ - '0');
        if (number > (0xFFFFFFFFUL - digit) / 10U) return 0U;
        number = number * 10U + digit;
    }
    *value = number; return 1U;
}
static uint8_t App_UARTPattern(uint16_t index)
{
    return (uint8_t)(index * 73U + (uint8_t)(uart_test_id >> (8U * (index & 3U))) + 0x5AU);
}
static void App_UARTFinish(const char *result)
{
    /* One correlated result, not ADC/random data. Includes UART HW flags. */
    (void)snprintf(reply, sizeof(reply), "UART_TEST,%lu,%s,%u,%u,%u,%lu\n",
        (unsigned long)uart_test_id, result, (unsigned)APP_UART_TEST_BYTES,
        (unsigned)uart_test_received, (unsigned)uart_test_mismatches,
        (unsigned long)UART1_GetHwErrorFlags());
    reply_length = (uint16_t)strlen(reply);
    reply_link = APP_LINK_USB;
    demo_state = APP_IDLE;
}
static void App_UARTStart(uint32_t id)
{
    uint16_t i;
    uart_test_id = id;
    uart_test_received = uart_test_mismatches = 0U;
    uart_command_length = 0U; uart_command_overflow = 0U;
    monitor_enabled = 0U;
    demo_state = APP_UART_TEST;
    /* Lazy initialization keeps the established CDC startup path unchanged. */
    demo_uart_status = UART1_Init(115200U);
    if (demo_uart_status != UART1_OK) { App_UARTFinish("INITERR"); return; }
    for (i = 0U; i < APP_UART_TEST_BYTES; ++i) uart_test_payload[i] = App_UARTPattern(i);
    demo_uart_status = UART1_Write(uart_test_payload, APP_UART_TEST_BYTES);
    if (demo_uart_status != UART1_OK) { App_UARTFinish("TXERR"); return; }
    uart_test_started = HAL_GetTick();
}
static void App_ProcessUARTTest(void)
{
    uint8_t bytes[32];
    uint16_t count, i;
    if (demo_state != APP_UART_TEST || reply_length) return;
    count = UART1_Read(bytes, sizeof(bytes));
    for (i = 0U; i < count; ++i) {
        if (uart_test_received >= APP_UART_TEST_BYTES ||
            bytes[i] != App_UARTPattern(uart_test_received)) {
            if (uart_test_mismatches < APP_UART_TEST_BYTES + 1U) ++uart_test_mismatches;
        }
        if (uart_test_received < APP_UART_TEST_BYTES + 1U) ++uart_test_received;
    }
    demo_uart_status = UART1_GetStatus();
    if (demo_uart_status != UART1_OK) App_UARTFinish("HWERR");
    else if (uart_test_received >= APP_UART_TEST_BYTES && UART1_IsTxIdle())
        App_UARTFinish((uart_test_received == APP_UART_TEST_BYTES && !uart_test_mismatches) ? "PASS" : "MISMATCH");
    else if ((uint32_t)(HAL_GetTick() - uart_test_started) >= APP_UART_TEST_TIMEOUT_MS)
        App_UARTFinish("TIMEOUT");
}
static void App_Start(void)
{
    uint64_t rate;
    if (demo_state != APP_IDLE) { App_Reply("ERR=BUSY\n"); return; }
    if (config_mode) { App_Reply("ERR=CONFIG_INCOMPLETE\n"); return; }
    if (rng9_selected && config.sample_count > APP_RAM_SAMPLES && config.adc_rate > APP_RNG9_MAX_RATE_HZ) {
        App_Reply("ERR=RNG_STREAM_RATE_MAX_10000\n"); return;
    }
    if (active_link == APP_LINK_UART && config.adc_rate > APP_UART_MAX_RATE_HZ) {
        App_Reply("ERR=UART_RATE_MAX_3000\n"); return;
    }
    if (!App_LinkConfigured(active_link)) return;
    monitor_enabled = post_processing = 0U;
    Timer_Stop(); ADC_StopDMA(); USB_CDC_ClearStatus();
    demo_received_samples = demo_sent_samples = demo_first_sample_index = 0U;
    demo_fifo_peak = demo_copy_cycles_max = demo_pack_cycles_max = demo_rng_slice_cycles_max = 0U;
    capture_elapsed_ms = 0U;
    data_packet_count = 0U;
    demo_error_code = 0U;
    demo_capture_done = demo_tx_complete = demo_done = demo_failed = 0U;
    active_count = config.sample_count;
    rng9_running = rng9_selected;
    RNG9_Reset(&rng9_state);
    stream_mode = (uint8_t)(active_count > APP_RAM_SAMPLES);
    /* Long captures are multiples of 250: constant DMA service interval,
     * exact N, no gaps or tiny divisor-dependent half buffers. */
    dma_block_count = stream_mode ? APP_DMA_HALF_SAMPLES : (uint16_t)(active_count / 2U);
    ++demo_session_id;
    /* ADC clock stays at 12 MHz (APB2/6). At 200 kHz a timer period
     * is only 5 us, so 71.5 ADC cycles is NOT safe. Keep the original
     * 71.5 cycles for <=100 kHz, reduce progressively for higher rates.
     * Do NOT reduce for high impedance sources without checking settling. */
    demo_status = ADC_SetSampleTimeBits(config.adc_rate <= 100000U ? 6U :
        config.adc_rate <= 150000U ? 5U :
        config.adc_rate <= 175000U ? 4U : 3U);
    if (demo_status != ADC_OK) { App_Fail(1U); return; }
    demo_timer_status = Timer_Init(config.adc_rate);
    rate = Timer_GetActualFrequencyMilliHz();
    if (demo_timer_status != TIMER_OK || rate == 0U || rate > 0xFFFFFFFFULL || !App_ADCPeriodIsValid()) {
        App_Fail(2U); return;
    }
    Timer_SetDebugFreeze(1U);
    demo_rate_millihz = (uint32_t)rate;
    capture_timeout = (uint32_t)(((uint64_t)active_count * 1000000U + rate - 1U) / rate) + 1000U;
    demo_status = ADC_StartDMA(ADC_CH_PA0, adc_dma_buffer, (uint16_t)(dma_block_count * 2U),
                               stream_mode ? ADC_DMA_CIRCULAR : ADC_DMA_NORMAL);
    if (demo_status != ADC_OK) { App_Fail(1U); return; }
    demo_timer_status = Timer_Start();
    if (demo_timer_status != TIMER_OK) { App_Fail(2U); return; }
    capture_started = last_tx_progress = HAL_GetTick();
    capture_active = 1U;
    demo_state = stream_mode ? APP_BEGIN : APP_CAPTURE;
    App_Reply("START_OK\n");
    App_Log(stream_mode ? "START STREAM\r\n" : "START RAM\r\n");
}
/* Buffered RNG9: acquire at the requested ADC rate, then replay the PC's
 * verified RAW capture through the exact same MCU SINE9 engine. The PC sends
 * one block (POST_BIN <=500 binary samples, or legacy POST_DATA <=23 text
 * samples) and waits for all of its RNG output before sending another.
 * No ADC/Timer is running during replay; processing speed never changes fs. */
static void App_PostBegin(const char *text)
{
    uint32_t source;
    if (active_link != APP_LINK_USB) { App_Reply("ERR=POST_USE_CDC\n"); return; }
    if (demo_state == APP_DRAIN) { App_Reply("ERR=POST_DRAINING\n"); return; }
    if (demo_state != APP_IDLE || config_mode) { App_Reply("ERR=POST_BUSY\n"); return; }
    if (!App_ParseU32(text, &source) || source != demo_session_id ||
        !demo_done || !demo_capture_done || rng9_running || post_processing ||
        demo_received_samples != active_count || demo_sent_samples != active_count) {
        App_Reply("ERR=POST_SOURCE\n"); return;
    }
    ++demo_session_id;
    demo_received_samples = demo_sent_samples = 0U;
    data_packet_count = 0U; post_bin_count = 0U;
    demo_fifo_peak = demo_copy_cycles_max = demo_pack_cycles_max = demo_rng_slice_cycles_max = 0U;
    demo_done = demo_tx_complete = demo_failed = 0U; demo_error_code = 0U;
    monitor_enabled = capture_active = 0U; rng9_running = post_processing = 1U;
    RNG9_Reset(&rng9_state);
    post_rx_tick = last_tx_progress = HAL_GetTick();
    demo_state = APP_BEGIN;
    App_Reply("POST_OK\n");
}
static void App_PostData(const char *text)
{
    uint32_t first = 0U, value;
    uint16_t count = 0U, digits = 0U, i;
    const char *cursor = text;
    if (!post_processing || active_link != APP_LINK_USB || demo_state != APP_DATA) {
        App_Reply("ERR=POST_STATE\n"); return;
    }
    if (demo_sent_samples != demo_received_samples || data_packet_count != 0U) {
        App_Reply("ERR=POST_BUSY\n"); return;
    }
    while (*cursor >= '0' && *cursor <= '9') {
        if (++digits > 7U) goto bad;
        first = first * 10U + (uint32_t)(*cursor++ - '0');
    }
    if (digits == 0U || first != demo_received_samples || *cursor++ != ',') goto bad;
    for (;;) {
        value = 0U; digits = 0U;
        while (*cursor >= '0' && *cursor <= '9') {
            if (++digits > 4U) goto bad;
            value = value * 10U + (uint32_t)(*cursor++ - '0');
        }
        if (digits == 0U || value > 4095U || count >= APP_BINARY_SAMPLES) goto bad;
        adc_block[count++] = (uint16_t)value;
        if (*cursor == '\0') break;
        if (*cursor++ != ',') goto bad;
    }
    if (count > active_count - demo_received_samples) goto bad;
    for (i = 0U; i < count; ++i) captured_samples[(first + i) % APP_FIFO_SAMPLES] = adc_block[i];
    demo_received_samples += count;
    if (count > demo_fifo_peak) demo_fifo_peak = count;
    post_rx_tick = HAL_GetTick();
    return;
bad:
    App_Fail(14U); /* Reject malformed/repeated/reordered input atomically. */
}
/* POST_BIN=<firstIndex>,<count>\n is followed by exactly count LE uint16
 * ADC values (0..4095). The header is checked before any payload byte is
 * stored; the block is committed to SINE9 only after its last byte. */
static void App_PostBin(const char *text)
{
    uint32_t first = 0U, count = 0U;
    uint16_t digits = 0U;
    const char *cursor = text;
    if (!post_processing || active_link != APP_LINK_USB || demo_state != APP_DATA) {
        App_Reply("ERR=POST_STATE\n"); return;
    }
    if (demo_sent_samples != demo_received_samples || data_packet_count != 0U || post_bin_count != 0U) {
        App_Reply("ERR=POST_BUSY\n"); return;
    }
    while (*cursor >= '0' && *cursor <= '9') {
        if (++digits > 7U) goto bad;
        first = first * 10U + (uint32_t)(*cursor++ - '0');
    }
    if (digits == 0U || first != demo_received_samples || *cursor++ != ',') goto bad;
    digits = 0U;
    while (*cursor >= '0' && *cursor <= '9') {
        if (++digits > 3U) goto bad;
        count = count * 10U + (uint32_t)(*cursor++ - '0');
    }
    if (digits == 0U || *cursor != '\0' || count == 0U || count > APP_POST_BIN_SAMPLES ||
        count > active_count - demo_received_samples) goto bad;
    post_bin_first = first; post_bin_count = (uint16_t)count; post_bin_bytes = 0U;
    post_rx_tick = HAL_GetTick();
    return;
bad:
    App_Fail(14U);
}
/* Bulk-read the pending POST_BIN payload straight into the sample FIFO.
 * Stop-and-wait keeps every slot it touches already sent (sent == received). */
static void App_ReceivePostBinary(void)
{
    uint8_t bytes[APP_POST_BIN_READ_BYTES];
    uint16_t count, i, total = (uint16_t)(post_bin_count * 2U);
    uint8_t chunks = 4U;
    while (chunks-- != 0U && post_bin_bytes < total) {
        uint16_t wanted = (uint16_t)(total - post_bin_bytes);
        count = USB_CDC_Read(bytes, wanted > sizeof(bytes) ? (uint16_t)sizeof(bytes) : wanted);
        if (count == 0U) return;
        post_rx_tick = HAL_GetTick();
        for (i = 0U; i < count; ++i, ++post_bin_bytes) {
            uint16_t *slot = &captured_samples[(post_bin_first + post_bin_bytes / 2U) % APP_FIFO_SAMPLES];
            if ((post_bin_bytes & 1U) == 0U) *slot = bytes[i];
            else if (bytes[i] > 0x0FU) { App_Fail(14U); return; } /* ADC > 4095 or lost sync. */
            else *slot = (uint16_t)(*slot | ((uint16_t)bytes[i] << 8U));
        }
    }
    if (post_bin_bytes == total) {
        demo_received_samples += post_bin_count;
        if (post_bin_count > demo_fifo_peak) demo_fifo_peak = post_bin_count;
        post_bin_count = 0U;
    }
}
static void App_ProcessLine(const char *line)
{
    uint32_t value;
    if (*line == '\0') return;
    /* Each CONFIG/capture belongs to its command port. A second client cannot
     * splice commands into it or redirect its data. STOP is owner-only too. */
    if ((demo_state != APP_IDLE || config_mode) && command_link != active_link) {
        App_Reply("ERR=BUSY_OTHER_LINK\n"); return;
    }
    if (demo_state == APP_IDLE && !config_mode) active_link = command_link;
    if (strcmp(line, "STOP") == 0) {
        config_mode = config_fields = monitor_enabled = 0U;
        Timer_Stop(); ADC_StopDMA(); capture_active = 0U;
        data_packet_count = 0U; post_processing = 0U; post_bin_count = 0U;
        if (demo_state != APP_IDLE) {
            demo_failed = 1U; demo_error_code = 9U; demo_done = demo_tx_complete = 0U;
        }
        /* Already queued DATA stays valid until its USB transfer finishes.
         * STOP_OK is ordered after it; no old DATA is queued after STOP_OK. */
        reply_length = 0U; demo_state = APP_STOP_ACK; last_tx_progress = HAL_GetTick();
        App_Log("STOP\r\n"); return;
    }
    if (strncmp(line, "POST_BEGIN=", 11U) == 0) { App_PostBegin(line + 11U); return; }
    if (strncmp(line, "POST_DATA=", 10U) == 0) { App_PostData(line + 10U); return; }
    if (strncmp(line, "POST_BIN=", 9U) == 0) { App_PostBin(line + 9U); return; }
    if (strcmp(line, "START") == 0) { App_Start(); return; }
    if (demo_state != APP_IDLE) { App_Reply("ERR=BUSY\n"); return; }
    if (strcmp(line, "GET_INFO") == 0) {
        App_Reply(active_link == APP_LINK_UART ?
            "INFO=ADC_BIN_3_2;HZ=3000;LINK=UART;RNG9=1;PACK9=1;POST=0\n" :
            "INFO=ADC_BIN_3_2;HZ=200000;LINK=USB;RNG9=1;PACK9=1;POST=2;F=3\n"); return;
    }
    if (strncmp(line, "UART_TEST=", 10U) == 0) {
        if (active_link != APP_LINK_USB) { App_Reply("ERR=UART_TEST_USE_CDC\n"); return; }
        if (config_mode) { App_Reply("ERR=CONFIG_INCOMPLETE\n"); return; }
        if (!App_ParseU32(line + 10U, &value) || !value) { App_Reply("ERR=UART_TEST_ID\n"); return; }
        App_UARTStart(value); return;
    }
    if (strcmp(line, "MONITOR=1") == 0 || strcmp(line, "MONITOR=0") == 0) {
        if (config_mode) { App_Reply("ERR=CONFIG_INCOMPLETE\n"); return; }
        monitor_enabled = (uint8_t)(line[8] == '1'); monitor_tick = HAL_GetTick();
        App_Reply("MONITOR_OK\n"); return;
    }
    if (strcmp(line, "CMD=CONFIG") == 0) {
        memset(&draft_config, 0, sizeof(draft_config));
        config_fields = 0U; config_mode = 1U; config_started = HAL_GetTick();
        rng9_draft = 0U; return;
    }
    if (strcmp(line, "GET_RANDOM") == 0) {
        App_Reply("ERR=RAW_ADC_USE_START\n"); return;
    }
    if (!config_mode) { App_Reply("ERR=UNKNOWN_COMMAND\n"); return; }
    if (strncmp(line, "ADC_RATE=", 9U) == 0) {
        uint32_t max_rate = active_link == APP_LINK_UART ? APP_UART_MAX_RATE_HZ : APP_MAX_RATE_HZ;
        if (!App_ParseU32(line + 9, &value) || value < 1U || value > max_rate || (config_fields & 1U)) {
            App_Reply(active_link == APP_LINK_UART ? "ERR=UART_RATE_MAX_3000\n" : "ERR=ADC_RATE_1_TO_200000\n"); goto invalid_config;
        }
        draft_config.adc_rate = value; config_fields |= 1U;
    }
    else if (strncmp(line, "SAMPLE=", 7U) == 0) {
        if (!App_ParseU32(line + 7, &value) || value < 2U || value > APP_MAX_SAMPLES || (value & 1U) || (config_fields & 2U)) {
            App_Reply("ERR=SAMPLE_EVEN_2_TO_1000000\n"); goto invalid_config;
        }
        draft_config.sample_count = value; config_fields |= 2U;
    }
    else if (strcmp(line, "MODE=RNG9") == 0 && config_fields == 3U) {
        rng9_draft = 1U; config_fields |= 8U;
    }
    else if (strncmp(line, "SIZE=", 5U) == 0) {
        if (!App_ParseU32(line + 5, &value) || (value != 32U && value != 64U && value != 128U) || (config_fields != 3U && config_fields != 11U)) {
            App_Reply("ERR=SIZE_LAST_32_64_128\n"); goto invalid_config;
        }
        draft_config.random_size = (uint16_t)value; config_fields |= 4U;
    }
    else { App_Reply("ERR=UNKNOWN_COMMAND\n"); goto invalid_config; }
    if (config_fields == 7U || config_fields == 15U) {
        if (rng9_draft && draft_config.sample_count > APP_RAM_SAMPLES && draft_config.adc_rate > APP_RNG9_MAX_RATE_HZ) {
            App_Reply("ERR=RNG_STREAM_RATE_MAX_10000\n"); goto invalid_config;
        }
        if (draft_config.sample_count > APP_RAM_SAMPLES &&
            draft_config.sample_count % APP_DMA_HALF_SAMPLES != 0U) {
            App_Reply("ERR=SAMPLE_STREAM_MULTIPLE_250\n"); goto invalid_config;
        }
        if (draft_config.sample_count > APP_RAM_SAMPLES && draft_config.adc_rate > APP_STREAM_MAX_RATE) {
            App_Reply("ERR=STREAM_RATE_MAX_200000\n"); goto invalid_config;
        }
        config = draft_config; rng9_selected = rng9_draft; config_mode = 0U;
        App_Reply("CONFIG_OK\n"); App_Log("CONFIG_OK\r\n");
    }
    return;
invalid_config:
    config_mode = config_fields = 0U;
}
static void App_ProcessLinkEvents(void)
{
    uint32_t generation = USB_CDC_GetLinkGeneration();
    if (generation != seen_link_generation) {
        seen_link_generation = generation;
        command_length = 0U; command_overflow = 0U; post_bin_count = 0U;
        if (reply_link == APP_LINK_USB) reply_length = 0U;
        if (active_link == APP_LINK_USB) {
            config_mode = monitor_enabled = 0U;
            if (demo_state != APP_IDLE && demo_state != APP_ERROR) App_Fail(11U);
        }
        USB_CDC_ClearStatus();
    }
}
static void App_ProcessCommands(void)
{
    uint8_t byte, budget, port;
    if (config_mode && (uint32_t)(HAL_GetTick() - config_started) >= APP_CONFIG_TIMEOUT_MS)
        config_mode = config_fields = 0U;
    for (port = 0U; port < 2U; ++port) {
        AppLink link = (AppLink)port;
        uint16_t *length = link == APP_LINK_USB ? &command_length : &uart_command_length;
        uint8_t *overflow = link == APP_LINK_USB ? &command_overflow : &uart_command_overflow;
        char *buffer = link == APP_LINK_USB ? command : uart_command;
        uint8_t rx_error = link == APP_LINK_USB ? USB_CDC_RxOverflow() :
            (uint8_t)(UART1_GetStatus() != UART1_OK && UART1_GetStatus() != UART1_ERROR_NOT_INITIALIZED);
        if (rx_error) {
            *length = 0U; *overflow = 1U;
            if (link == active_link) {
                command_link = link; config_mode = 0U;
                if (demo_state != APP_IDLE && demo_state != APP_UART_TEST) App_Fail(12U);
                else if (demo_state == APP_IDLE) App_Reply("ERR=RX_OVERFLOW\n");
            }
            if (link == APP_LINK_UART && demo_state != APP_UART_TEST) UART1_ClearStatus();
            continue;
        }
        /* The loopback test itself owns UART RX, and the second PC cannot
         * delay a capture by flooding commands through the other port. */
        if (demo_state == APP_UART_TEST && link == APP_LINK_UART) continue;
        if ((demo_state != APP_IDLE || config_mode) && link != active_link) continue;
        if (link == APP_LINK_USB && post_bin_count != 0U) {
            App_ReceivePostBinary();
            if (post_bin_count != 0U) continue;
        }
        budget = 64U;
        while (budget-- != 0U && reply_length == 0U &&
            (link == APP_LINK_USB ? USB_CDC_Read(&byte, 1U) : UART1_Read(&byte, 1U)) != 0U) {
            /* Read can itself discover a DMA overwrite. Never execute bytes
             * returned by that failed read as a fresh command. */
            if (link == APP_LINK_UART && UART1_GetStatus() != UART1_OK) {
                *length = 0U; *overflow = 1U; break;
            }
            if (byte == '\r') continue;
            if (byte == '\n') {
                buffer[*length] = '\0'; command_link = link;
                if (!*overflow) App_ProcessLine(buffer);
                else { config_mode = 0U; App_Reply("ERR=BAD_OR_LONG_LINE\n"); }
                *length = 0U; *overflow = 0U;
                if (post_bin_count != 0U) break; /* Payload bytes follow; never parse them as text. */
            }
            else if (!*overflow && byte >= 32U && byte <= 126U && *length < (link == APP_LINK_USB ? sizeof(command) : sizeof(uart_command)) - 1U)
                buffer[(*length)++] = (char)byte;
            else *overflow = 1U;
        }
    }
}
static void App_ProcessReply(void)
{
    USB_CDC_Status_t status;
    if (reply_length == 0U) return;
    status = App_LinkWrite(reply_link, (const uint8_t *)reply, reply_length);
    if (status != USB_CDC_BUSY) reply_length = 0U;
}
static void App_ProcessCapture(void)
{
    uint16_t count, i;
    uint32_t index, pending, cycles_started;
    ADC_Status_t result;
    if (!capture_active) return;
    cycles_started = App_Cycles();
    result = ADC_DMA_CopyBlock(adc_block, APP_BLOCK_CAPACITY, &count, &index);
    if (result == ADC_OK) {
        if (index != demo_received_samples || count != dma_block_count ||
            demo_received_samples + count > active_count) { App_Fail(3U); return; }
        pending = demo_received_samples - demo_sent_samples + count;
        if (pending > APP_FIFO_SAMPLES) { App_Fail(13U); return; }
        if (pending > demo_fifo_peak) demo_fifo_peak = pending;
        for (i = 0U; i < count; ++i)
            captured_samples[(index + i) % APP_FIFO_SAMPLES] = adc_block[i];
        demo_first_sample_index = index; demo_received_samples += count;
        if (cycle_counter_available) {
            uint32_t elapsed = App_Cycles() - cycles_started;
            if (elapsed > demo_copy_cycles_max) demo_copy_cycles_max = elapsed;
        }
    }
    else if (result != ADC_NO_DATA) { demo_status = result; App_Fail(4U); return; }
    if (demo_received_samples == active_count) {
        Timer_Stop(); ADC_StopDMA(); capture_active = 0U; demo_capture_done = 1U;
        capture_elapsed_ms = HAL_GetTick() - capture_started;
        if (!stream_mode) { last_tx_progress = HAL_GetTick(); demo_state = APP_BEGIN; }
    }
    else if ((uint32_t)(HAL_GetTick() - capture_started) > capture_timeout) App_Fail(5U);
}
static void App_ProcessTx(void)
{
    AppState state = demo_state;
    USB_CDC_Status_t status;
    int length = 0;
    if (state == APP_IDLE || state == APP_CAPTURE || state == APP_UART_TEST) return;
    if (post_processing && state == APP_DATA && demo_sent_samples == demo_received_samples &&
        demo_received_samples < active_count && (uint32_t)(HAL_GetTick() - post_rx_tick) > APP_TX_TIMEOUT_MS) {
        App_Fail(8U); return;
    }
    if (reply_length != 0U) {
        if ((uint32_t)(HAL_GetTick() - last_tx_progress) > APP_TX_TIMEOUT_MS) App_Fail(8U);
        return;
    }
    if ((state == APP_ERROR || state == APP_STOP_ACK) && !App_LinkConfigured(active_link)) { demo_state = APP_IDLE; return; }
    if (state == APP_DRAIN || state == APP_ABORT_DRAIN) {
        if (App_LinkTxIdle(active_link)) {
            if (state == APP_DRAIN) { post_processing = 0U; demo_tx_complete = demo_done = 1U; App_Log("END SENT\r\n"); }
            demo_state = APP_IDLE;
        }
    }
    else {
        if (state == APP_BEGIN)
            length = snprintf(tx_line, sizeof(tx_line), rng9_running ? "BEGINR,%lu,%lu,%lu\n" : "BEGINB,%lu,%lu,%lu\n", (unsigned long)demo_session_id,
                (unsigned long)active_count, (unsigned long)demo_rate_millihz);
        else if (state == APP_DATA) {
            if (demo_sent_samples == demo_received_samples) {
                /* Waiting for the next DMA half, including low sample rates.
                 * Capture timeout owns this wait, not the USB TX watchdog. */
                last_tx_progress = HAL_GetTick(); return;
            }
            /* Carry the tail across DMA halves to fill packets. Flush the
             * final partial packet after capture ends. BUSY keeps ownership. */
            if (data_packet_count == 0U && capture_active &&
                demo_received_samples - demo_sent_samples <
                    (rng9_running ? APP_RNG_PACKED_SAMPLES :
                     (active_link == APP_LINK_USB ? APP_RAW_FAST_SAMPLES : APP_BINARY_SAMPLES))) {
                last_tx_progress = HAL_GetTick(); return;
            }
            if ((data_packet_count == 0U || data_packet_length == 0U) && !App_PrepareDataPacket()) return;
            length = (int)data_packet_length;
        }
        else if (state == APP_END)
            length = snprintf(tx_line, sizeof(tx_line), "END,%lu,%lu\n", (unsigned long)demo_session_id,
                (unsigned long)active_count);
        else if (state == APP_METRICS)
            length = snprintf(tx_line, sizeof(tx_line), "MTR,%lu,%lu,%lu,%lu,%lu,%lu\n",
                (unsigned long)demo_session_id, (unsigned long)demo_fifo_peak,
                (unsigned long)demo_copy_cycles_max, (unsigned long)demo_pack_cycles_max,
                (unsigned long)capture_elapsed_ms, (unsigned long)SystemCoreClock);
        else if (state == APP_ERROR)
            length = snprintf(tx_line, sizeof(tx_line), "ERROR,%lu,%lu\n", (unsigned long)demo_session_id,
                (unsigned long)demo_error_code);
        else if (state == APP_STOP_ACK)
            length = snprintf(tx_line, sizeof(tx_line), "STOP_OK\n");
        if (length <= 0 || length > (int)sizeof(tx_line) ||
            (state != APP_DATA && length == (int)sizeof(tx_line))) { App_Fail(6U); return; }
        status = App_LinkWrite(active_link, (const uint8_t *)tx_line, (uint16_t)length);
        if (status == USB_CDC_OK) {
            last_tx_progress = HAL_GetTick();
            if (state == APP_BEGIN) demo_state = APP_DATA;
            else if (state == APP_DATA) {
                demo_sent_samples += data_packet_count;
                if (rng9_running) rng9_state = rng9_packet_state;
                data_packet_count = 0U;
                if (demo_sent_samples == active_count) { demo_state = APP_METRICS; }
            }
            else if (state == APP_METRICS) demo_state = APP_END;
            else if (state == APP_END) demo_state = APP_DRAIN;
            else if (state == APP_ERROR || state == APP_STOP_ACK) demo_state = APP_ABORT_DRAIN;
        }
        else if (status != USB_CDC_BUSY) {
            if (state == APP_ERROR || state == APP_STOP_ACK) demo_state = APP_IDLE;
            else App_Fail(7U);
        }
    }
    if ((uint32_t)(HAL_GetTick() - last_tx_progress) > APP_TX_TIMEOUT_MS) {
        if (demo_state == APP_ERROR || demo_state == APP_ABORT_DRAIN) demo_state = APP_IDLE;
        else if (demo_state != APP_IDLE) App_Fail(8U);
    }
}
static void App_ProcessMonitor(void)
{
    uint16_t value;
    int length;
    uint32_t now = HAL_GetTick();
    if (!monitor_enabled || demo_state != APP_IDLE || config_mode || reply_length ||
        !App_LinkTxIdle(active_link) || (uint32_t)(now - monitor_tick) < 50U) return;
    monitor_tick = now;
    demo_status = ADC_Read(&value);
    if (demo_status != ADC_OK) { monitor_enabled = 0U; App_Reply("ERR=MONITOR_ADC\n"); return; }
    length = snprintf(tx_line, sizeof(tx_line), "ADC=%lu,%u\n", (unsigned long)(now * 1000U), (unsigned)value);
    if (length > 0 && length < (int)sizeof(tx_line))
        (void)App_LinkWrite(active_link, (const uint8_t *)tx_line, (uint16_t)length);
}
static void App_Init(void)
{
    /* Cortex-M3 DWT measures real work on the board, including preemption.
     * Zero metrics mean the counter is unavailable; not a CPU benchmark. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    cycle_counter_available = (uint8_t)((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0U);
    if (cycle_counter_available) { DWT->CYCCNT = 0U; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; }
    /* USART1 is a second PC transport, not a log sink. CDC enumeration remains
     * independent of UART readiness; GPIO and DMA clocks are enabled locally. */
    demo_uart_status = UART1_Init(115200U);
    demo_status = ADC_Init(ADC_CH_PA0);
    demo_timer_status = Timer_Init(config.adc_rate);
    Timer_SetDebugFreeze(1U);
    seen_link_generation = USB_CDC_GetLinkGeneration();
    if (demo_status != ADC_OK || demo_timer_status != TIMER_OK || !App_ADCPeriodIsValid()) App_Fail(10U);
    demo_boot_stage = 5U;
}
/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

/* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  demo_boot_stage = 1U;
/* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  demo_boot_stage = 2U;
  USB_CDC_App_Init(); /* Before USB endpoints can call our hooks. */
  demo_boot_stage = 3U;
/* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  demo_boot_stage = 4U;
  App_Init();
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    USB_CDC_App_Task();
    App_ProcessLinkEvents();
    /* Retire END/STOP transfers before accepting the next CONFIG. */
    if (demo_state == APP_DRAIN || demo_state == APP_ABORT_DRAIN) App_ProcessTx();
    App_ProcessCommands();
    if (demo_state != APP_IDLE && demo_state != APP_ERROR && demo_state != APP_ABORT_DRAIN && demo_state != APP_STOP_ACK &&
        (!App_LinkConfigured(active_link) || !App_LinkHealthy(active_link))) App_Fail(11U);
    App_ProcessCapture();
    App_ProcessUARTTest();
    App_ProcessReply();
    App_ProcessTx();
    App_ProcessMonitor();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
