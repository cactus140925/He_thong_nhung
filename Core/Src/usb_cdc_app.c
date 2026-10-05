#include "usb_cdc_app.h"

#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usbd_cdc.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>


extern USBD_HandleTypeDef hUsbDeviceFS;

#define CDC_LINE_BUFFER_SIZE    96
#define CDC_TX_BUFFER_SIZE      128



static TRNG_Config_t trng_config;



static char line_buffer[CDC_LINE_BUFFER_SIZE];

static uint16_t line_index = 0;



static uint8_t config_mode = 0;

static uint8_t config_adc_received = 0;
static uint8_t config_sample_received = 0;
static uint8_t config_size_received = 0;



static uint8_t trng_running = 0;



static volatile CDC_Event_t pending_event =
    CDC_EVENT_NONE;



static uint8_t tx_buffer[CDC_TX_BUFFER_SIZE];




static void ProcessLine(char *line);

static uint8_t USB_CDC_WaitTxReady(
    uint32_t timeout_ms
);

static void CheckConfigComplete(void);



void USB_CDC_App_Init(void)
{
    memset(&trng_config, 0, sizeof(trng_config));

    line_index = 0;

    config_mode = 0;

    config_adc_received = 0;
    config_sample_received = 0;
    config_size_received = 0;

    trng_running = 0;

    pending_event = CDC_EVENT_NONE;
}




static uint8_t USB_CDC_WaitTxReady(
    uint32_t timeout_ms
)
{
    uint32_t start = HAL_GetTick();

    while (1)
    {
        if (hUsbDeviceFS.dev_state ==
            USBD_STATE_CONFIGURED)
        {
            if (hUsbDeviceFS.pClassData != NULL)
            {
                USBD_CDC_HandleTypeDef *hcdc =
                    (USBD_CDC_HandleTypeDef *)
                    hUsbDeviceFS.pClassData;

                if (hcdc->TxState == 0)
                {
                    return 1;
                }
            }
        }

        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return 0;
        }
    }
}


void USB_CDC_SendString(const char *str)
{
    uint16_t len;

    if (str == NULL)
    {
        return;
    }

    len = strlen(str);

    if (len >= CDC_TX_BUFFER_SIZE)
    {
        len = CDC_TX_BUFFER_SIZE - 1;
    }


    if (!USB_CDC_WaitTxReady(100))
    {
        return;
    }

    memcpy(tx_buffer, str, len);

    CDC_Transmit_FS(
        tx_buffer,
        len
    );
}




void USB_CDC_SendRandomResult(
    const uint8_t *random_data,
    uint16_t random_bits,
    uint32_t elapsed_ms,
    uint32_t sample_count
)
{
    char text[96];

    char hex[40];

    uint16_t bytes;
    uint16_t i;
    uint16_t pos = 0;

    if (random_data == NULL)
    {
        return;
    }


    if ((random_bits != 32) &&
        (random_bits != 64) &&
        (random_bits != 128))
    {
        USB_CDC_SendString(
            "ERROR:INVALID_RANDOM_SIZE\r\n"
        );

        return;
    }


    bytes = random_bits / 8;


    /*
     * Binary -> HEX
     */
    for (i = 0; i < bytes; i++)
    {
        pos += snprintf(
            &hex[pos],
            sizeof(hex) - pos,
            "%02X",
            random_data[i]
        );
    }

    hex[pos] = '\0';


    /*
     * DATA
     */
    snprintf(
        text,
        sizeof(text),
        "DATA=%s\r\n",
        hex
    );

    USB_CDC_SendString(text);


    /*
     * TIME
     */
    snprintf(
        text,
        sizeof(text),
        "TIME=%lu\r\n",
        (unsigned long)elapsed_ms
    );

    USB_CDC_SendString(text);


    /*
     * SAMPLE
     */
    snprintf(
        text,
        sizeof(text),
        "SAMPLE=%lu\r\n",
        (unsigned long)sample_count
    );

    USB_CDC_SendString(text);
}




static void CheckConfigComplete(void)
{
    if (!config_mode)
    {
        return;
    }


    if (config_adc_received &&
        config_sample_received &&
        config_size_received)
    {
        trng_config.valid = 1;

        config_mode = 0;

        USB_CDC_SendString(
            "CONFIG_OK\r\n"
        );
    }
}




static void ProcessLine(char *line)
{
    unsigned long value;


    /*
     * Empty line
     */
    if (line[0] == '\0')
    {
        return;
    }



    if (strcmp(line, "CMD=CONFIG") == 0)
    {
        memset(
            &trng_config,
            0,
            sizeof(trng_config)
        );

        config_mode = 1;

        config_adc_received = 0;
        config_sample_received = 0;
        config_size_received = 0;

        return;
    }




    if (strncmp(
            line,
            "ADC_RATE=",
            9) == 0)
    {
        if (!config_mode)
        {
            USB_CDC_SendString(
                "ERROR:CONFIG_NOT_STARTED\r\n"
            );

            return;
        }

        value =
            strtoul(
                &line[9],
                NULL,
                10
            );


        if (value == 0)
        {
            USB_CDC_SendString(
                "ERROR:INVALID_ADC_RATE\r\n"
            );

            return;
        }


        trng_config.adc_rate = value;

        config_adc_received = 1;

        CheckConfigComplete();

        return;
    }



    if (strncmp(
            line,
            "SAMPLE=",
            7) == 0)
    {
        if (!config_mode)
        {
            USB_CDC_SendString(
                "ERROR:CONFIG_NOT_STARTED\r\n"
            );

            return;
        }


        value =
            strtoul(
                &line[7],
                NULL,
                10
            );


        if (value == 0)
        {
            USB_CDC_SendString(
                "ERROR:INVALID_SAMPLE\r\n"
            );

            return;
        }


        trng_config.sample_count = value;

        config_sample_received = 1;

        CheckConfigComplete();

        return;
    }



    if (strncmp(
            line,
            "SIZE=",
            5) == 0)
    {
        if (!config_mode)
        {
            USB_CDC_SendString(
                "ERROR:CONFIG_NOT_STARTED\r\n"
            );

            return;
        }


        value =
            strtoul(
                &line[5],
                NULL,
                10
            );


        if ((value != 32) &&
            (value != 64) &&
            (value != 128))
        {
            USB_CDC_SendString(
                "ERROR:SIZE_MUST_BE_32_64_128\r\n"
            );

            return;
        }


        trng_config.random_size =
            (uint16_t)value;

        config_size_received = 1;

        CheckConfigComplete();

        return;
    }




    if (strcmp(line, "START") == 0)
    {
        if (!trng_config.valid)
        {
            USB_CDC_SendString(
                "ERROR:NOT_CONFIGURED\r\n"
            );

            return;
        }


        trng_running = 1;

        pending_event =
            CDC_EVENT_START;


        USB_CDC_SendString(
            "START_OK\r\n"
        );

        return;
    }




    if (strcmp(line, "STOP") == 0)
    {
        trng_running = 0;

        pending_event =
            CDC_EVENT_STOP;


        USB_CDC_SendString(
            "STOP_OK\r\n"
        );

        return;
    }




    if (strcmp(line, "GET_RANDOM") == 0)
    {
        if (!trng_running)
        {
            USB_CDC_SendString(
                "ERROR:NOT_STARTED\r\n"
            );

            return;
        }


        pending_event =
            CDC_EVENT_GET_RANDOM;

        return;
    }




    USB_CDC_SendString(
        "ERROR:UNKNOWN_COMMAND\r\n"
    );
}



void USB_CDC_App_Task(void)
{
    uint8_t rx[32];

    uint16_t count;
    uint16_t i;

    if (USB_CDC_RxOverflow())
    {
        line_index = 0;

        USB_CDC_SendString(
            "ERROR:RX_OVERFLOW\r\n"
        );
    }


    count =
        USB_CDC_Read(
            rx,
            sizeof(rx)
        );


    for (i = 0; i < count; i++)
    {
        char c = (char)rx[i];

        if (c == '\r')
        {
            continue;
        }



        if (c == '\n')
        {
            line_buffer[line_index] = '\0';

            ProcessLine(line_buffer);

            line_index = 0;

            continue;
        }


        if (line_index <
            (CDC_LINE_BUFFER_SIZE - 1))
        {
            line_buffer[line_index++] = c;
        }
        else
        {

            line_index = 0;

            USB_CDC_SendString(
                "ERROR:LINE_TOO_LONG\r\n"
            );
        }
    }
}




const TRNG_Config_t *USB_CDC_GetConfig(void)
{
    return &trng_config;
}


CDC_Event_t USB_CDC_GetEvent(void)
{
    CDC_Event_t event;

    event = pending_event;

    pending_event = CDC_EVENT_NONE;

    return event;
}