#include "usb_cdc_app.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
extern USBD_HandleTypeDef hUsbDeviceFS;
typedef struct { uint8_t bytes[USB_CDC_TX_SLOT_BYTES]; uint16_t length; } USB_CDC_TxSlot;
static USB_CDC_TxSlot tx_slots[USB_CDC_TX_SLOTS];
static volatile uint8_t tx_head, tx_tail, tx_count, in_flight;
static volatile USB_CDC_Status_t transport_status;
static volatile uint32_t link_generation;
static uint8_t initialized;
static uint32_t USB_CDC_Lock(void)
{ uint32_t mask = __get_PRIMASK(); __disable_irq(); __DMB(); return mask; }
static void USB_CDC_Unlock(uint32_t mask) { __DMB(); __set_PRIMASK(mask); }
static uint8_t USB_CDC_ConfiguredLocked(void)
{ return (uint8_t)(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL); }
void USB_CDC_App_Init(void)
{
    uint32_t mask = USB_CDC_Lock();
    tx_head = tx_tail = tx_count = in_flight = 0U;
    transport_status = USB_CDC_OK; link_generation = 0U; initialized = 1U;
    USB_CDC_Unlock(mask);
}
void USB_CDC_OnLinkReset(void)
{
    /* Interface Init/DeInit runs after endpoint reset/close. Release old slots
     * only there, never merely because the host stopped reading. */
    uint32_t mask = USB_CDC_Lock();
    tx_head = tx_tail = tx_count = in_flight = 0U;
    transport_status = USB_CDC_LINK_RESET; ++link_generation;
    USB_CDC_Unlock(mask);
}
uint8_t USB_CDC_IsConfigured(void)
{
    uint8_t ready;
    uint32_t mask = USB_CDC_Lock();
    ready = initialized ? USB_CDC_ConfiguredLocked() : 0U;
    USB_CDC_Unlock(mask); return ready;
}
USB_CDC_Status_t USB_CDC_TxEnqueue(const uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint32_t mask;
    if (data == NULL || length == 0U || length > USB_CDC_TX_SLOT_BYTES) return USB_CDC_ERROR_PARAM;
    mask = USB_CDC_Lock();
    if (!initialized || !USB_CDC_ConfiguredLocked()) { USB_CDC_Unlock(mask); return USB_CDC_NOT_CONFIGURED; }
    if (tx_count == USB_CDC_TX_SLOTS) { USB_CDC_Unlock(mask); return USB_CDC_BUSY; }
    for (i = 0U; i < length; ++i) tx_slots[tx_tail].bytes[i] = data[i];
    tx_slots[tx_tail].length = length;
    tx_tail = (uint8_t)((tx_tail + 1U) % USB_CDC_TX_SLOTS); ++tx_count;
    USB_CDC_Unlock(mask); return USB_CDC_OK;
}
USB_CDC_Status_t USB_CDC_SendString(const char *text)
{
    size_t length;
    if (text == NULL) return USB_CDC_ERROR_PARAM;
    length = strlen(text);
    if (length == 0U || length > USB_CDC_TX_SLOT_BYTES) return USB_CDC_ERROR_PARAM;
    return USB_CDC_TxEnqueue((const uint8_t *)text, (uint16_t)length);
}
void USB_CDC_App_Task(void)
{
    uint8_t result;
    uint32_t mask;
    USBD_CDC_HandleTypeDef *handle;
    if (!initialized) return;
    mask = USB_CDC_Lock();
    if (!USB_CDC_ConfiguredLocked()) { USB_CDC_Unlock(mask); return; }
    handle = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (in_flight && handle->TxState == 0U) {
        tx_head = (uint8_t)((tx_head + 1U) % USB_CDC_TX_SLOTS); --tx_count; in_flight = 0U;
    }
    if (!in_flight && tx_count != 0U && handle->TxState == 0U) {
        result = CDC_Transmit_FS(tx_slots[tx_head].bytes, tx_slots[tx_head].length);
        if (result == USBD_OK) in_flight = 1U;
        else if (result != USBD_BUSY) transport_status = USB_CDC_ERROR_USB;
    }
    USB_CDC_Unlock(mask);
}
uint8_t USB_CDC_IsTxIdle(void)
{
    uint8_t idle = 0U;
    uint32_t mask = USB_CDC_Lock();
    if (initialized && USB_CDC_ConfiguredLocked()) {
        USBD_CDC_HandleTypeDef *handle = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
        idle = (uint8_t)(tx_count == 0U && !in_flight && handle->TxState == 0U);
    }
    USB_CDC_Unlock(mask); return idle;
}
USB_CDC_Status_t USB_CDC_GetStatus(void) { return transport_status; }
void USB_CDC_ClearStatus(void)
{ uint32_t mask = USB_CDC_Lock(); transport_status = USB_CDC_OK; USB_CDC_Unlock(mask); }
uint32_t USB_CDC_GetLinkGeneration(void) { return link_generation; }
