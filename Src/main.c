#include "main.h"
#include "dmo_data.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usbd_def.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"
#include "stm32f1xx_hal_iwdg.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint32_t marker;
    uint32_t selected_sku_index;
} FlashStorage;

/* Private define ------------------------------------------------------------*/
#define FLASH_STORAGE_PAGE_ADDR 0x0800FC00 // Use the last page of 64K flash
#define FLASH_VALID_MARKER      0xDEADBEEF

#define SLIX2_BLOCKS         80
#define SLIX2_INVENTORY_LEN   9
#define SLIX2_SYSINFO_LEN    14
#define SLIX2_NXPSYSINFO_LEN  7
#define SLIX2_SIGNATURE_LEN  32

#define I2C_RCV_BUF_SIZE 257
#define I2C_SND_BUF_SIZE 257

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
IWDG_HandleTypeDef hiwdg;

extern USBD_HandleTypeDef hUsbDeviceFS;
volatile uint32_t current_sku_index = 0;
uint32_t label_counter = 0;
volatile uint32_t reset_countdown = 0;

/* I2C Slave Emulation Variables */
static uint8_t  EMU_SLIX2_INVENTORY[SLIX2_INVENTORY_LEN];
static uint8_t  EMU_SLIX2_SYSINFO[SLIX2_SYSINFO_LEN];
static uint8_t  EMU_SLIX2_NXPSYSINFO[SLIX2_NXPSYSINFO_LEN];
static uint8_t  EMU_SLIX2_SIGNATURE[SLIX2_SIGNATURE_LEN];
static uint32_t EMU_SLIX2_BLOCKS[SLIX2_BLOCKS];
static uint16_t EMU_SLIX2_COUNTER;
static bool     EMU_SLIX2_TAG_PRESENT;

static uint8_t I2CSlaveRecvBuf[I2C_RCV_BUF_SIZE];
static uint8_t I2CSlaveRecvBufLen;
static uint8_t I2CSlaveSendBuf[I2C_SND_BUF_SIZE];
static uint8_t I2CSlaveSendBufLen;

/* EEPROM Emulation at address 0x50 */
static uint8_t eeprom_data[1024]; // 1KB EEPROM emulation
static uint16_t eeprom_address = 0; // Current address pointer
static uint8_t address_bytes_received = 0; // Track address byte reception

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_IWDG_Init(void);
void FLASH_LoadDefaultSKUIndex(void);

/* I2C Emulation Function Prototypes */
void EMU_SLIX2_CounterReset(void);
void EMU_SLIX2_Communication(const uint8_t* pindata, const uint8_t inlength, uint8_t* poutdata, uint8_t* poutlength);
void EMU_CLRC688_IRQSet(uint8_t irq);
void EMU_CLRC688_Communication(const uint8_t* pindata, const uint8_t inlength, uint8_t* poutdata, uint8_t* poutlength);
void InitEmulationWithCurrentSKU(void);
void UpdateEmulationDataForSKUChange(void);

/* Main user code ---------------------------------------------------------*/
void send_current_sku_message() {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "\r\n--- free-dmo-stm32 ---\r\nCurrently emulating: %s\r\n", dmo_skus[current_sku_index].sku_name);
    send_string_to_usb(buffer);
}

void send_prompt() {
    send_string_to_usb("> ");
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  FLASH_LoadDefaultSKUIndex();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_IWDG_Init();
  MX_USB_DEVICE_Init();

  label_counter = dmo_skus[current_sku_index].label_count;

  // Initialize EEPROM emulation with SLIX2 tag data
  InitEmulationWithCurrentSKU();
  send_string_to_usb("[EEPROM] SLIX2 tag emulation initialized with authentic data\r\n");
  char sku_msg[128];
  snprintf(sku_msg, sizeof(sku_msg), "[EEPROM] Current SKU: %s (%d labels)\r\n",
           dmo_skus[current_sku_index].sku_name, dmo_skus[current_sku_index].label_count);
  send_string_to_usb(sku_msg);

  send_string_to_usb("[EEPROM] Enabling I2C EEPROM emulation at address 0x50...\r\n");
  HAL_StatusTypeDef i2c_status = HAL_I2C_EnableListen_IT(&hi2c1);
  if (i2c_status == HAL_OK) {
    send_string_to_usb("[EEPROM] I2C EEPROM emulation enabled successfully\r\n");
  } else {
    char status_msg[64];
    snprintf(status_msg, sizeof(status_msg), "[EEPROM] Failed to enable I2C listen mode: %d\r\n", i2c_status);
    send_string_to_usb(status_msg);
  }
  uint8_t message_sent = 0;
  uint32_t connection_check_counter = 0;

  while (1)
  {
	  // Check for USB connection periodically, even if not detected yet
	  connection_check_counter++;
	  if (connection_check_counter > 1000) { // Every ~1 second
		  connection_check_counter = 0;
		  if (!g_usb_is_connected) {
			  // Try to detect if USB is actually connected by checking device state
			  extern USBD_HandleTypeDef hUsbDeviceFS;
			  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
				  g_usb_is_connected = 1;
			  }
		  }
	  }

	  if (g_usb_is_connected) {
		  if (!message_sent) {
			  HAL_Delay(100); // Small delay to ensure USB is ready
			  send_current_sku_message();
			  send_prompt();
			  message_sent = 1;
		  }

		  USB_Service_Transmit();

		  if (g_reset_pending && is_usb_tx_buffer_empty()) {
			  // Start reset countdown
			  if (reset_countdown == 0) {
				  reset_countdown = 100; // 100ms countdown
			  }
		  }

		  // Handle reset countdown - aggressive USB disconnect for macOS
		  if (reset_countdown > 0) {
			  reset_countdown--;
			  if (reset_countdown == 0) {
				  // Multiple aggressive approaches to force USB disconnect on macOS

				  // 1. Disable USB interrupts first
				  NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

				  // 2. Stop USB device completely
				  extern USBD_HandleTypeDef hUsbDeviceFS;
				  USBD_Stop(&hUsbDeviceFS);

				  // 3. Force USB peripheral reset at RCC level
				  SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USBRST);

				  // 4. Disable USB clock
				  __HAL_RCC_USB_CLK_DISABLE();

				  // 5. Configure USB pins as outputs and drive them to unusual state
				  GPIO_InitTypeDef GPIO_InitStruct = {0};

				  // PA11 (USB_DM) - drive high instead of low
				  GPIO_InitStruct.Pin = GPIO_PIN_11;
				  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				  GPIO_InitStruct.Pull = GPIO_NOPULL;
				  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
				  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

				  // PA12 (USB_DP) - drive high too (invalid USB state)
				  GPIO_InitStruct.Pin = GPIO_PIN_12;
				  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

				  // 6. Wait for macOS to detect the invalid USB state
				  HAL_Delay(100);

				  // 7. Create a more gradual disconnect to reduce terminal corruption
				  // First, pull D+ low while keeping D- high (SE0 state)
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
				  HAL_Delay(50);

				  // Then pull D- low too (full disconnect)
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

				  // 8. Final delay for clean disconnect
				  HAL_Delay(100);

				  // 9. Finally reset
				  HAL_NVIC_SystemReset();
			  }
		  }
	  } else {
		  message_sent = 0;
		  g_reset_pending = 0;
		  reset_countdown = 0;
	  }

      // I2C emulation is handled by interrupt callbacks
	  HAL_Delay(1);

	  // Refresh watchdog to prevent reset during normal operation
	  HAL_IWDG_Refresh(&hiwdg);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 80; // I2C slave address for CLRC688 emulation
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095; // ~4 second timeout
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

void FLASH_LoadDefaultSKUIndex(void)
{
    FlashStorage stored_data = *(FlashStorage*)FLASH_STORAGE_PAGE_ADDR;

    if (stored_data.marker == FLASH_VALID_MARKER && stored_data.selected_sku_index < dmo_skus_count)
    {
        current_sku_index = stored_data.selected_sku_index;
    }
    else
    {
        current_sku_index = 0;
        for (uint32_t i = 0; i < dmo_skus_count; i++) {
            if(strcmp(dmo_skus[i].sku_name, "DMO_SKU_30334") == 0) {
                current_sku_index = i;
                break;
            }
        }
    }
}

void FLASH_WriteDefaultSKUIndex(uint32_t index)
{
    if (index >= dmo_skus_count) return;

    FlashStorage data_to_write;
    data_to_write.marker = FLASH_VALID_MARKER;
    data_to_write.selected_sku_index = index;

    HAL_FLASH_Unlock();
    __disable_irq();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_STORAGE_PAGE_ADDR;
    EraseInitStruct.NbPages = 1;

    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_PAGE_ADDR, data_to_write.marker);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_PAGE_ADDR + 4, data_to_write.selected_sku_index);

    __enable_irq();
    HAL_FLASH_Lock();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

/* I2C Emulation Functions */
void EMU_SLIX2_CounterReset(void) {
  uint16_t amount_of_labels = EMU_SLIX2_BLOCKS[0x0F]>>16;
  uint16_t counter_margin = EMU_SLIX2_BLOCKS[0x10]>>16;
  EMU_SLIX2_COUNTER = 0xFFFF - amount_of_labels - counter_margin;
}

void EMU_SLIX2_Communication(const uint8_t* pindata, const uint8_t inlength, uint8_t* poutdata, uint8_t* poutlength) {
  switch(pindata[1]) { //emulate command for tag
    case 0x01: { //inventory
      EMU_SLIX2_CounterReset();
      poutdata[0]=0;
      memcpy(poutdata+1, EMU_SLIX2_INVENTORY, SLIX2_INVENTORY_LEN);
      *poutlength = 1 + SLIX2_INVENTORY_LEN;
      EMU_SLIX2_TAG_PRESENT = true;
      break;
    }
    case 0x21: { //write single block
      if( 0x4F == pindata[10] ) { //write counter
        if( (1==pindata[11]) && (0==pindata[12]) && (0==pindata[13]) && (0==pindata[14]) ) { //only allow increment by 1
          EMU_SLIX2_COUNTER++;
        }
        else {
          poutdata[0]=0x01; poutdata[1]=0x0F; *poutlength=2; //set error
          break;
        }
      }
      poutdata[0]=0; *poutlength=1;
      break;
    }
    case 0x23: { //read multiple block
      uint8_t blk = pindata[10];
      if( blk >= SLIX2_BLOCKS ) {
        poutdata[0]=0x01; poutdata[1]=0x0F; *poutlength=2; //set error
        break;
      }
      uint8_t cnt = pindata[11]+1;
      if( blk+cnt > SLIX2_BLOCKS )
        cnt = SLIX2_BLOCKS-blk;
      poutdata[0]=0;
      memcpy(poutdata+1, &EMU_SLIX2_BLOCKS[blk], cnt*4);
      *poutlength = 1 + cnt*4;

      //special case counter
      if( (79==blk) || (blk+cnt>=79) ) {
        poutdata[1 + (79-blk)*4 + 0] = (uint8_t)EMU_SLIX2_COUNTER;
        poutdata[1 + (79-blk)*4 + 1] = (uint8_t)(EMU_SLIX2_COUNTER>>8);
        poutdata[1 + (79-blk)*4 + 2] = 0;
        poutdata[1 + (79-blk)*4 + 3] = 1;
      }
      break;
    }
    case 0x26: { //reset to ready (used to check if tag still present)
      if( EMU_SLIX2_TAG_PRESENT ) {
        poutdata[0]=0; *poutlength=1;
      } else {
        poutdata[0]=0x01; poutdata[1]=0x0F; *poutlength=2; //set error
      }
      break;
    }
    case 0x2B: poutdata[0]=0; memcpy(poutdata+1, EMU_SLIX2_SYSINFO, SLIX2_SYSINFO_LEN); *poutlength=1+SLIX2_SYSINFO_LEN; break;                          //sysinfo
    case 0xAB: poutdata[0]=0; memcpy(poutdata+1, EMU_SLIX2_NXPSYSINFO, SLIX2_NXPSYSINFO_LEN); *poutlength=1+SLIX2_NXPSYSINFO_LEN; break;                 //nxp sysinfo
    case 0xB2: poutdata[0]=0; poutdata[1]=HAL_GetTick(); poutdata[2]=HAL_GetTick()>>8; *poutlength=3; break;                                             //random
    case 0xB3: poutdata[0]=0; *poutlength=1; break;                                                                                                      //set password => just signal success
    case 0xBD: poutdata[0]=0; memcpy(poutdata+1, EMU_SLIX2_SIGNATURE, SLIX2_SIGNATURE_LEN); *poutlength=1+SLIX2_SIGNATURE_LEN; break;                    //signature
    default:   poutdata[0]=0; *poutlength=1; break;                                                                                                      //always signal success
  }
}

void EMU_CLRC688_IRQSet(uint8_t irq) {
  // Note: GPIO pins need to be defined in main.h for OUT_IRQ_GPIO_Port and OUT_IRQ_Pin
  // For now, this is a placeholder - implement based on your hardware configuration
  // HAL_GPIO_WritePin(OUT_IRQ_GPIO_Port, OUT_IRQ_Pin, irq?GPIO_PIN_SET:GPIO_PIN_RESET);
}

void EMU_CLRC688_Communication(const uint8_t* pindata, const uint8_t inlength, uint8_t* poutdata, uint8_t* poutlength) {
  static uint8_t clrc668_fifo_buffer[256];
  static uint8_t clrc668_fifo_length = 0;

  // Debug logging
  char debug_msg[128];
  if (inlength > 0) {
    snprintf(debug_msg, sizeof(debug_msg), "[CLRC688] RX: len=%d data=", inlength);
    send_string_to_usb(debug_msg);
    for (int i = 0; i < inlength && i < 8; i++) {
      snprintf(debug_msg, sizeof(debug_msg), "0x%02X ", pindata[i]);
      send_string_to_usb(debug_msg);
    }
    send_string_to_usb("\r\n");
  }

  *poutlength = 0;
  if( inlength>0 ) {
    switch( pindata[0] ) {
      case 0x00: { //command for reader ic
        if (inlength >= 2) {
          char cmd_debug[64];
          snprintf(cmd_debug, sizeof(cmd_debug), "[CLRC688] CMD: 0x00 0x%02X\r\n", pindata[1]);
          send_string_to_usb(cmd_debug);

          switch( pindata[1] ) {
            case 0x07: {  //handle xfer to_/from tag
              send_string_to_usb("[CLRC688] RF Transfer triggered!\r\n");
              EMU_SLIX2_Communication(clrc668_fifo_buffer, clrc668_fifo_length, clrc668_fifo_buffer, &clrc668_fifo_length);
              EMU_CLRC688_IRQSet(1);   //signal IRQ
              break;
            }
            case 0x40: {
              send_string_to_usb("[CLRC688] Power/config command 0x40\r\n");
              break;
            }
            default: {
              char default_debug[64];
              snprintf(default_debug, sizeof(default_debug), "[CLRC688] Unknown 0x00 cmd: 0x%02X\r\n", pindata[1]);
              send_string_to_usb(default_debug);
              break;
            }
          }
          if( pindata[1] & 0x80 ) //reset the counter when software standby mode is entered
            EMU_SLIX2_CounterReset();
        }
        break;
      }

      case 0x04: { //fifolen
        if( 1 == inlength ) {
          poutdata[0] = clrc668_fifo_length;
          *poutlength = 1;
        }
        break;
      }

      case 0x05: { //fifodata
        if( inlength>1 ) { //incoming data - command to tag
          memcpy(clrc668_fifo_buffer, pindata+1, inlength-1);
          clrc668_fifo_length = inlength-1;

          // Process DYMO RFID commands and generate appropriate responses
          if (clrc668_fifo_length >= 4) {
            uint8_t cmd = clrc668_fifo_buffer[0];
            char cmd_debug[128];
            snprintf(cmd_debug, sizeof(cmd_debug), "[DYMO-RFID] Processing cmd=0x%02X (len=%d): ", cmd, clrc668_fifo_length);
            send_string_to_usb(cmd_debug);
            for (int i = 0; i < clrc668_fifo_length && i < 8; i++) {
              char byte_str[8];
              snprintf(byte_str, sizeof(byte_str), "0x%02X ", clrc668_fifo_buffer[i]);
              send_string_to_usb(byte_str);
            }
            send_string_to_usb("\r\n");

            switch (cmd) {
              case 0x36: {
                // ISO15693 Inventory command - Find DYMO SLIX2 tag
                send_string_to_usb("[DYMO-RFID] INVENTORY: Printer scanning for authentic DYMO tag\r\n");

                // Response: [response_flags] + [UID] (ISO15693 format)
                clrc668_fifo_buffer[0] = 0x00; // Response flags (no error)
                // Copy our authentic SLIX2 UID (from authentic DMO tag data)
                clrc668_fifo_buffer[1] = 0xBA; clrc668_fifo_buffer[2] = 0x6C;
                clrc668_fifo_buffer[3] = 0x60; clrc668_fifo_buffer[4] = 0x3D;
                clrc668_fifo_buffer[5] = 0x08; clrc668_fifo_buffer[6] = 0x01;
                clrc668_fifo_buffer[7] = 0x04; clrc668_fifo_buffer[8] = 0xE0;
                clrc668_fifo_length = 9;

                EMU_SLIX2_TAG_PRESENT = true;
                send_string_to_usb("[DYMO-RFID] RESPONSE: Sending authentic DYMO SLIX2 UID\r\n");
                break;
              }

              default: {
                // Handle other DYMO RFID operations (block reads, counter operations, etc.)
                snprintf(cmd_debug, sizeof(cmd_debug), "[DYMO-RFID] Unknown/Unhandled command: 0x%02X\r\n", cmd);
                send_string_to_usb(cmd_debug);
                // For now, just echo back success
                clrc668_fifo_buffer[0] = 0x00; // Success response
                clrc668_fifo_length = 1;
                break;
              }
            }

            // Signal that data is ready
            EMU_CLRC688_IRQSet(1);
            char ready_debug[64];
            snprintf(ready_debug, sizeof(ready_debug), "[DYMO-RFID] Response ready in FIFO (%d bytes)\r\n", clrc668_fifo_length);
            send_string_to_usb(ready_debug);
          }
        }
        else { //outgoing data - read FIFO
          if( clrc668_fifo_length ) {
            memcpy(poutdata, clrc668_fifo_buffer, clrc668_fifo_length);
            char fifo_read_debug[128];
            snprintf(fifo_read_debug, sizeof(fifo_read_debug), "[CLRC688] FIFO READ: sending %d bytes: ", clrc668_fifo_length);
            send_string_to_usb(fifo_read_debug);
            for (int i = 0; i < clrc668_fifo_length && i < 8; i++) {
              char byte_debug[8];
              snprintf(byte_debug, sizeof(byte_debug), "0x%02X ", poutdata[i]);
              send_string_to_usb(byte_debug);
            }
            send_string_to_usb("\r\n");
          }
          *poutlength = clrc668_fifo_length;
          clrc668_fifo_length = 0;
        }
        break;
      }

      case 0x06: //irq0 register
      case 0x07: //irq1 register
      {
        if( 1 == inlength ) { //is it a read?
          poutdata[0] = 0x7F; //signal all ints
          *poutlength = 1;
        }
        else
          EMU_CLRC688_IRQSet(0);  //clear interrupts / stop signalling IRQ
        break;
      }

      case 0x0A: { //error register
        poutdata[0] = 0x00; //no error
        *poutlength = 1;
        break;
      }

      case 0x02: // CLRC688 register
      case 0x2C: // CLRC688 register
      case 0x2F: // CLRC688 register
      case 0x33: // CLRC688 register
      {
        if (inlength == 1) {
          // Register read
          poutdata[0] = 0x00;
          *poutlength = 1;
        } else {
          // Register write - no response needed
          *poutlength = 0;
        }
        break;
      }

      default: {
        // Handle other register reads/writes
        if (inlength == 1) {
          // Single byte commands - register reads
          switch (pindata[0]) {
            case 0x0C: poutdata[0] = 0x00; *poutlength = 1; break;
            case 0x2B: poutdata[0] = 0x05; *poutlength = 1; break;
            case 0x32: poutdata[0] = 0x00; *poutlength = 1; break;
            case 0x39: poutdata[0] = 0x00; *poutlength = 1; break;
            case 0x0B: poutdata[0] = 0x00; *poutlength = 1; break;
            case 0x0F: poutdata[0] = 0x00; *poutlength = 1; break;
            case 0x08: poutdata[0] = 0x00; *poutlength = 1; break;
            case 0x11: poutdata[0] = 0x00; *poutlength = 1; break;
            default:
              poutdata[0] = 0x00;
              *poutlength = 1;
              break;
          }
        } else if (inlength == 2) {
          // Two byte commands - register writes
          *poutlength = 0;
        }
        break;
      }
    }
  }

  // Debug logging for output
  if (*poutlength > 0) {
    snprintf(debug_msg, sizeof(debug_msg), "[CLRC688] TX: len=%d data=", *poutlength);
    send_string_to_usb(debug_msg);
    for (int i = 0; i < *poutlength && i < 8; i++) {
      snprintf(debug_msg, sizeof(debug_msg), "0x%02X ", poutdata[i]);
      send_string_to_usb(debug_msg);
    }
    send_string_to_usb("\r\n");
  } else {
    send_string_to_usb("[CLRC688] TX: no response\r\n");
  }
}

void InitEmulationWithCurrentSKU(void) {
  // Initialize EEPROM data with authentic SLIX2 tag layout
  // Clear EEPROM first
  memset(eeprom_data, 0, sizeof(eeprom_data));

  // Use authentic SLIX2 tag data from FreeDMO repository
  // Based on DMO_SKU_30252 working data

  // SLIX2 tag blocks data (80 blocks of 4 bytes each = 320 bytes starting at address 0x00)
  const uint32_t authentic_blocks[SLIX2_BLOCKS] = {
    0xed820a03, 0xd2613986, 0x321e1403, 0x3c00cab6, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x015e0000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00010000
  };

  // Copy blocks data to EEPROM (little-endian format)
  for (int i = 0; i < SLIX2_BLOCKS; i++) {
    uint32_t block = authentic_blocks[i];
    eeprom_data[i*4 + 0] = (block >> 0) & 0xFF;
    eeprom_data[i*4 + 1] = (block >> 8) & 0xFF;
    eeprom_data[i*4 + 2] = (block >> 16) & 0xFF;
    eeprom_data[i*4 + 3] = (block >> 24) & 0xFF;
  }

  // Adjust label count for current SKU at block 0x0F (offset 0x3C in EEPROM)
  const dmo_sku_t* current_sku = &dmo_skus[current_sku_index];
  uint32_t label_count_block = (uint32_t)current_sku->label_count << 16;
  eeprom_data[0x3C] = (label_count_block >> 0) & 0xFF;
  eeprom_data[0x3D] = (label_count_block >> 8) & 0xFF;
  eeprom_data[0x3E] = (label_count_block >> 16) & 0xFF;
  eeprom_data[0x3F] = (label_count_block >> 24) & 0xFF;

  // Reset EEPROM address pointer
  eeprom_address = 0;
  address_bytes_received = 0;

  // Debug: Show first few bytes of EEPROM data
  char eeprom_debug[128];
  snprintf(eeprom_debug, sizeof(eeprom_debug), "[EEPROM] First 16 bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
           eeprom_data[0], eeprom_data[1], eeprom_data[2], eeprom_data[3], eeprom_data[4], eeprom_data[5], eeprom_data[6], eeprom_data[7],
           eeprom_data[8], eeprom_data[9], eeprom_data[10], eeprom_data[11], eeprom_data[12], eeprom_data[13], eeprom_data[14], eeprom_data[15]);
  send_string_to_usb(eeprom_debug);
}

void UpdateEmulationDataForSKUChange(void) {
  // This function should be called whenever the SKU is changed
  // It reinitializes the emulation data with the new SKU
  InitEmulationWithCurrentSKU();
}

/* I2C Slave Callback Functions - DYMO RFID Emulation */
static uint32_t transaction_counter = 0;

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transferDirection, uint16_t addrMatchCode) {
  transaction_counter++;

  send_string_to_usb("\r\n=== TX START ===\r\n");
  char debug_msg[64];
  snprintf(debug_msg, sizeof(debug_msg), "[TX#%lu] dir=%d\r\n", transaction_counter, transferDirection);
  send_string_to_usb(debug_msg);

  if (transferDirection == I2C_DIRECTION_RECEIVE) {
    // Master wants to read from us - process previous command and send response
    snprintf(debug_msg, sizeof(debug_msg), "[I2C] READ - Processing %d received bytes\r\n", I2CSlaveRecvBufLen);
    send_string_to_usb(debug_msg);

    // Use CLRC688 communication to generate response
    EMU_CLRC688_Communication(I2CSlaveRecvBuf, I2CSlaveRecvBufLen, I2CSlaveSendBuf, &I2CSlaveSendBufLen);

    snprintf(debug_msg, sizeof(debug_msg), "[I2C] SENDING %d bytes\r\n", I2CSlaveSendBufLen);
    send_string_to_usb(debug_msg);

    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, I2CSlaveSendBuf, I2CSlaveSendBufLen, I2C_LAST_FRAME);
  } else {
    // Master wants to write to us - receive command/data
    I2CSlaveRecvBufLen = 0;
    send_string_to_usb("[I2C] WRITE - Receiving command\r\n");
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, I2CSlaveRecvBuf, 1, I2C_NEXT_FRAME);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  char debug_msg[64];
  uint8_t received_byte = I2CSlaveRecvBuf[I2CSlaveRecvBufLen];
  I2CSlaveRecvBufLen++;

  snprintf(debug_msg, sizeof(debug_msg), "[I2C] RX: 0x%02X (len=%d)\r\n", received_byte, I2CSlaveRecvBufLen);
  send_string_to_usb(debug_msg);

  // Continue receiving if buffer not full
  if (I2CSlaveRecvBufLen < I2C_RCV_BUF_SIZE) {
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, I2CSlaveRecvBuf + I2CSlaveRecvBufLen, 1, I2C_NEXT_FRAME);
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  send_string_to_usb("[I2C] Transmission complete\r\n");
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  char debug_msg[64];
  snprintf(debug_msg, sizeof(debug_msg), "[I2C] Listen complete, received %d bytes\r\n", I2CSlaveRecvBufLen);
  send_string_to_usb(debug_msg);

  // Process command with CLRC688 emulation for DYMO RFID operations
  EMU_CLRC688_Communication(I2CSlaveRecvBuf, I2CSlaveRecvBufLen, I2CSlaveSendBuf, &I2CSlaveSendBufLen);

  send_string_to_usb("=== TX END ===\r\n");

  // Re-enable listening for next transaction
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  char debug_msg[64];
  snprintf(debug_msg, sizeof(debug_msg), "[I2C] Error: state=0x%08X, error=0x%08X\r\n",
           (unsigned int)hi2c->State, (unsigned int)hi2c->ErrorCode);
  send_string_to_usb(debug_msg);

  // Check if this is just an AF (Acknowledge Failure) which is normal at end of read
  if (hi2c->ErrorCode == HAL_I2C_ERROR_AF) {
    send_string_to_usb("[I2C] AF error - normal end of transaction\r\n");
  }

  HAL_I2C_EnableListen_IT(hi2c);
}

