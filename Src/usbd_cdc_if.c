/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : VCP Interface file.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "main.h"
#include "dmo_data.h"
#include <string.h>
#include <stdio.h>

/* External function declarations */
extern void UpdateEmulationDataForSKUChange(void);

/* Private variables ---------------------------------------------------------*/
volatile uint8_t g_usb_is_connected = 0;
volatile uint8_t g_reset_pending = 0;

static uint8_t g_cmd_buffer[128];
static uint32_t g_cmd_buffer_index = 0;

#define TX_BUFFER_SIZE 1024
static uint8_t g_tx_buffer[TX_BUFFER_SIZE];
static volatile uint32_t g_tx_buffer_head = 0;
static volatile uint32_t g_tx_buffer_tail = 0;

/* Private function prototypes -----------------------------------------------*/
void process_command(void);
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USB Device Core handle declaration. */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Buffers for reception and transmission           */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];


USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  switch(cmd)
  {
    case CDC_SET_LINE_CODING:
        g_usb_is_connected = 1;
    break;

    case CDC_GET_LINE_CODING:
    break;

    case CDC_SET_CONTROL_LINE_STATE:
        // Check DTR (Data Terminal Ready) bit to determine connection state
        if (length >= 2) {
            uint16_t line_state = *((uint16_t*)pbuf);
            g_usb_is_connected = (line_state & 0x01) ? 1 : 0; // DTR bit
        } else {
            // If no data provided, assume connection
            g_usb_is_connected = 1;
        }
    break;
  }
  return (USBD_OK);
}

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  for (uint32_t i = 0; i < *Len; i++)
  {
    uint8_t received_char = Buf[i];
    if (received_char == '\r' || received_char == '\n')
    {
      if (g_cmd_buffer_index > 0)
      {
        send_string_to_usb("\r\n");
        g_cmd_buffer[g_cmd_buffer_index] = '\0';
        process_command();
        g_cmd_buffer_index = 0;
      } else {
        send_prompt();
      }
    }
    else if (received_char == '\b' || received_char == 127)
    {
      if (g_cmd_buffer_index > 0)
      {
        g_cmd_buffer_index--;
        send_string_to_usb("\b \b");
      }
    }
    else if (received_char >= ' ' && received_char <= '~')
    {
      if (g_cmd_buffer_index < sizeof(g_cmd_buffer) - 1)
      {
        g_cmd_buffer[g_cmd_buffer_index++] = received_char;
        char temp_char[2] = {received_char, '\0'};
        send_string_to_usb(temp_char);
      }
    }
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  return result;
}

uint8_t is_usb_tx_buffer_empty(void)
{
    return (g_tx_buffer_head == g_tx_buffer_tail);
}

void send_string_to_usb(const char* str)
{
    for (const char* p = str; *p != '\0'; p++)
    {
        g_tx_buffer[g_tx_buffer_head] = *p;
        g_tx_buffer_head = (g_tx_buffer_head + 1) % TX_BUFFER_SIZE;
    }
}

void USB_Service_Transmit()
{
    if (g_tx_buffer_head != g_tx_buffer_tail)
    {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        if (hcdc->TxState == 0)
        {
            uint16_t len = 0;
            if (g_tx_buffer_head > g_tx_buffer_tail)
            {
                len = g_tx_buffer_head - g_tx_buffer_tail;
            }
            else
            {
                len = TX_BUFFER_SIZE - g_tx_buffer_tail;
            }
            CDC_Transmit_FS(&g_tx_buffer[g_tx_buffer_tail], len);
            g_tx_buffer_tail = (g_tx_buffer_tail + len) % TX_BUFFER_SIZE;
        }
    }
}

void process_command(void)
{
    char* command = (char*)g_cmd_buffer;
    char response[256];

    if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0)
    {
        send_string_to_usb("Available commands:\r\n");
        send_string_to_usb("  help or ?       - Show this help\r\n");
        send_string_to_usb("  current         - Show current SKU\r\n");
        send_string_to_usb("  list            - List all available SKUs\r\n");
        send_string_to_usb("  changeto <SKU>  - Change to specified SKU\r\n");
        send_string_to_usb("  reset           - Restart the device\r\n");
    }
    else if (strcmp(command, "current") == 0 || strcmp(command, "status") == 0)
    {
        extern volatile uint32_t current_sku_index;
        extern const dmo_sku_t dmo_skus[];
        snprintf(response, sizeof(response), "Currently emulating: %s (%d labels)\r\n",
                 dmo_skus[current_sku_index].sku_name,
                 dmo_skus[current_sku_index].label_count);
        send_string_to_usb(response);
    }
    else if (strcmp(command, "list") == 0)
    {
        send_string_to_usb("Available SKUs:\r\n");
        for (uint32_t i = 0; i < dmo_skus_count; i++)
        {
            snprintf(response, sizeof(response), "- %s\r\n", dmo_skus[i].sku_name);
            send_string_to_usb(response);
        }
    }
    else if (strcmp(command, "reset") == 0)
    {
        send_string_to_usb("Disconnecting USB and resetting...\r\n");
        g_reset_pending = 1;
    }
    else if (strncmp(command, "changeto ", 9) == 0)
    {
        char* sku_name = command + 9;
        uint8_t found = 0;
        for (uint32_t i = 0; i < dmo_skus_count; i++)
        {
            if (strcmp(sku_name, dmo_skus[i].sku_name) == 0)
            {
                FLASH_WriteDefaultSKUIndex(i);
                // Update current SKU index and emulation data immediately
                extern volatile uint32_t current_sku_index;
                current_sku_index = i;
                UpdateEmulationDataForSKUChange();
                snprintf(response, sizeof(response), "OK. SKU changed to %.64s. Change applied immediately!\r\n", sku_name);
                send_string_to_usb(response);
                found = 1;
                break;
            }
        }
        if (!found)
        {
            snprintf(response, sizeof(response), "Error: SKU '%.64s' not found.\r\n", sku_name);
            send_string_to_usb(response);
        }
    }
    else
    {
        snprintf(response, sizeof(response), "Unknown command: '%.64s'\r\n", command);
        send_string_to_usb(response);
    }

    send_prompt();
}


