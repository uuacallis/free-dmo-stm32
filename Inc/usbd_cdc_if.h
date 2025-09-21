/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN EXPORTS */
// Global variables defined in usbd_cdc_if.c that main.c needs to see
extern volatile uint8_t g_usb_is_connected;
extern volatile uint8_t g_reset_pending;

// Functions defined in usbd_cdc_if.c that main.c needs to call
void send_string_to_usb(const char* str);
void USB_Service_Transmit(void);
uint8_t is_usb_tx_buffer_empty(void);
/* USER CODE END EXPORTS */

/** @defgroup USBD_CDC_IF_Exported_Defines */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

/** @defgroup USBD_CDC_IF_Exported_Types */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototypes */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);


#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */


