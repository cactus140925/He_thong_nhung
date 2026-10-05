#ifndef USB_CDC_APP_H
#define USB_CDC_APP_H

#include "main.h"
#include <stdint.h>

/*
 * C?u hình TRNG do PC g?i xu?ng.
 */
typedef struct
{
    uint32_t adc_rate;
    uint32_t sample_count;

    /*
     * 32 / 64 / 128 bit
     */
    uint16_t random_size;

    uint8_t valid;

} TRNG_Config_t;


/*
 * Các s? ki?n PC g?i cho MCU.
 */
typedef enum
{
    CDC_EVENT_NONE = 0,

    CDC_EVENT_START,
    CDC_EVENT_STOP,
    CDC_EVENT_GET_RANDOM

} CDC_Event_t;


/*
 * Kh?i t?o t?ng protocol.
 */
void USB_CDC_App_Init(void);


/*
 * G?i liên t?c trong while(1).
 */
void USB_CDC_App_Task(void);


/*
 * L?y c?u hình hi?n t?i.
 */
const TRNG_Config_t *USB_CDC_GetConfig(void);


/*
 * L?y command/event t? PC.
 */
CDC_Event_t USB_CDC_GetEvent(void);


/*
 * G?i text lên PC.
 */
void USB_CDC_SendString(const char *str);


/*
 * G?i k?t qu? random v? PC.
 *
 * random_data:
 *      d? li?u random binary
 *
 * random_bits:
 *      32 / 64 / 128
 *
 * elapsed_ms:
 *      th?i gian t?o random
 *
 * sample_count:
 *      s? m?u ADC dã s? d?ng
 */
void USB_CDC_SendRandomResult(
    const uint8_t *random_data,
    uint16_t random_bits,
    uint32_t elapsed_ms,
    uint32_t sample_count
);

#endif