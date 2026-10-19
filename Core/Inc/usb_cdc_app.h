#ifndef USB_CDC_APP_H
#define USB_CDC_APP_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Transport only. Protocol/acquisition/framing live in main.c.
 * One foreground caller; interface callbacks run in USB IRQ. */
#define USB_CDC_TX_SLOT_BYTES 64U
#define USB_CDC_TX_SLOTS 4U
typedef enum {
    USB_CDC_OK = 0, USB_CDC_BUSY, USB_CDC_ERROR_PARAM,
    USB_CDC_NOT_CONFIGURED, USB_CDC_ERROR_USB, USB_CDC_LINK_RESET
} USB_CDC_Status_t;
void USB_CDC_App_Init(void); /* Before MX_USB_DEVICE_Init, exactly once. */
void USB_CDC_App_Task(void); /* Nonblocking TX service. */
USB_CDC_Status_t USB_CDC_TxEnqueue(const uint8_t *data, uint16_t length);
USB_CDC_Status_t USB_CDC_SendString(const char *text);
uint8_t USB_CDC_IsConfigured(void);
uint8_t USB_CDC_IsTxIdle(void);
USB_CDC_Status_t USB_CDC_GetStatus(void);
void USB_CDC_ClearStatus(void);
uint32_t USB_CDC_GetLinkGeneration(void);
void USB_CDC_OnLinkReset(void); /* Only interface Init/DeInit. */
/* Implemented by the branch's usbd_cdc_if.c. */
uint16_t USB_CDC_Read(uint8_t *buffer, uint16_t max_len);
uint8_t USB_CDC_RxOverflow(void);
#ifdef __cplusplus
}
#endif
#endif
