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

  // Initialize I2C slave emulation
  InitEmulationWithCurrentSKU();
  send_string_to_usb("[I2C] Enabling I2C slave listen mode...\r\n");
  HAL_StatusTypeDef i2c_status = HAL_I2C_EnableListen_IT(&hi2c1);
  if (i2c_status == HAL_OK) {
    send_string_to_usb("[I2C] I2C slave listen mode enabled successfully\r\n");
  } else {
    char status_msg[64];
    snprintf(status_msg, sizeof(status_msg), "[I2C] Failed to enable I2C listen mode: %d\r\n", i2c_status);
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
    snprintf(debug_msg, sizeof(debug_msg), "[CLRC688] RX: cmd=0x%02X len=%d\r\n", pindata[0], inlength);
    send_string_to_usb(debug_msg);
  }

  *poutlength = 0;
  if( inlength>0 ) {
    switch( pindata[0] ) {
      case 0x00: { //command for reader ic
        switch( pindata[1] ) {
          case 0x07: {  //handle xfer to_/from tag
            EMU_SLIX2_Communication(clrc668_fifo_buffer, clrc668_fifo_length, clrc668_fifo_buffer, &clrc668_fifo_length);
            EMU_CLRC688_IRQSet(1);   //signal IRQ
            break;
          }
        }
        if( pindata[1] & 0x80 ) //reset the counter when software standby mode is entered
          EMU_SLIX2_CounterReset();
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
        if( inlength>1 ) { //incoming data
          memcpy(clrc668_fifo_buffer, pindata+1, inlength-1);
          clrc668_fifo_length = inlength-1;
        }
        else { //outgoing data
          if( clrc668_fifo_length )
            memcpy(poutdata, clrc668_fifo_buffer, clrc668_fifo_length);
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

      default: //ignore all other register writes
        break;
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
  }
}

void InitEmulationWithCurrentSKU(void) {
  // Initialize with default SLIX2 data for the current SKU
  // These are based on the FreeDMO repository default values

  // Default inventory data (UID pattern)
  const uint8_t default_inventory[SLIX2_INVENTORY_LEN] = {0x01,0xBA,0x6C,0x60,0x3D,0x08,0x01,0x04,0xE0};
  memcpy(EMU_SLIX2_INVENTORY, default_inventory, SLIX2_INVENTORY_LEN);

  // Default system info
  const uint8_t default_sysinfo[SLIX2_SYSINFO_LEN] = {0x0F,0xBA,0x6C,0x60,0x3D,0x08,0x01,0x04,0xE0,0x01,0x3D,0x4F,0x03,0x01};
  memcpy(EMU_SLIX2_SYSINFO, default_sysinfo, SLIX2_SYSINFO_LEN);

  // Default NXP system info
  const uint8_t default_nxpsysinfo[SLIX2_NXPSYSINFO_LEN] = {0x32,0x02,0x0F,0x7F,0x35,0x00,0x00};
  memcpy(EMU_SLIX2_NXPSYSINFO, default_nxpsysinfo, SLIX2_NXPSYSINFO_LEN);

  // Default signature
  const uint8_t default_signature[SLIX2_SIGNATURE_LEN] = {
    0x33,0x4A,0x63,0x63,0xD0,0x13,0x49,0xDB,0xA0,0x9E,0xEE,0x15,0x1E,0xF8,0xF8,0xF3,
    0xFA,0x15,0xF5,0x77,0xE4,0x4D,0x75,0x9B,0x78,0x14,0xCA,0xD3,0x7E,0x02,0xEF,0x10
  };
  memcpy(EMU_SLIX2_SIGNATURE, default_signature, SLIX2_SIGNATURE_LEN);

  // Default blocks data with current SKU label count
  memset(EMU_SLIX2_BLOCKS, 0, SLIX2_BLOCKS*sizeof(uint32_t));

  // DMO header and magic bytes
  EMU_SLIX2_BLOCKS[0] = 0xed820a03;
  EMU_SLIX2_BLOCKS[1] = 0xd2613986;
  EMU_SLIX2_BLOCKS[2] = 0x321e1403;
  EMU_SLIX2_BLOCKS[3] = 0x3c00cab6;

  // Set label count from current SKU
  const dmo_sku_t* current_sku = &dmo_skus[current_sku_index];
  EMU_SLIX2_BLOCKS[0x0F] = (uint32_t)current_sku->label_count << 16;
  EMU_SLIX2_BLOCKS[0x10] = 0x00000000; // Counter margin

  // Counter value (last block)
  EMU_SLIX2_BLOCKS[79] = 0x00010000; // Initial counter value

  // Reset counter
  EMU_SLIX2_CounterReset();
  EMU_SLIX2_TAG_PRESENT = false;
}

void UpdateEmulationDataForSKUChange(void) {
  // This function should be called whenever the SKU is changed
  // It reinitializes the emulation data with the new SKU
  InitEmulationWithCurrentSKU();
}

/* I2C Slave Callback Functions */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transferDirection, uint16_t addrMatchCode) {
  // Debug logging
  char debug_msg[64];
  snprintf(debug_msg, sizeof(debug_msg), "[I2C] Addr callback: dir=%d, addr=0x%X\r\n", transferDirection, addrMatchCode);
  send_string_to_usb(debug_msg);

  if (transferDirection == I2C_DIRECTION_RECEIVE) {
    // Master wants to receive data from us (read operation)
    EMU_CLRC688_Communication(I2CSlaveRecvBuf, I2CSlaveRecvBufLen, I2CSlaveSendBuf, &I2CSlaveSendBufLen);
    snprintf(debug_msg, sizeof(debug_msg), "[I2C] Sending %d bytes\r\n", I2CSlaveSendBufLen);
    send_string_to_usb(debug_msg);
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, I2CSlaveSendBuf, I2CSlaveSendBufLen, I2C_LAST_FRAME);
  } else {
    // Master wants to send data to us (write operation)
    I2CSlaveRecvBufLen = 0;
    send_string_to_usb("[I2C] Starting receive\r\n");
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, I2CSlaveRecvBuf, 1, I2C_NEXT_FRAME);
  }
  // Note: LED control requires GPIO configuration - placeholder for now
  // HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, GPIO_PIN_RESET);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  I2CSlaveRecvBufLen++;
  char debug_msg[64];
  snprintf(debug_msg, sizeof(debug_msg), "[I2C] Received byte: 0x%02X (len=%d)\r\n", I2CSlaveRecvBuf[I2CSlaveRecvBufLen-1], I2CSlaveRecvBufLen);
  send_string_to_usb(debug_msg);

  if( I2CSlaveRecvBufLen < I2C_RCV_BUF_SIZE )
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, I2CSlaveRecvBuf + I2CSlaveRecvBufLen, 1, I2C_NEXT_FRAME);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  send_string_to_usb("[I2C] Transmission complete\r\n");
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  char debug_msg[64];
  snprintf(debug_msg, sizeof(debug_msg), "[I2C] Listen complete, received %d bytes\r\n", I2CSlaveRecvBufLen);
  send_string_to_usb(debug_msg);

  EMU_CLRC688_Communication(I2CSlaveRecvBuf, I2CSlaveRecvBufLen, I2CSlaveSendBuf, &I2CSlaveSendBufLen);
  HAL_I2C_EnableListen_IT(hi2c);
  // Note: LED control requires GPIO configuration - placeholder for now
  // HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, GPIO_PIN_SET);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  char debug_msg[64];
  snprintf(debug_msg, sizeof(debug_msg), "[I2C] Error callback: state=0x%08X, error=0x%08X\r\n",
           (unsigned int)hi2c->State, (unsigned int)hi2c->ErrorCode);
  send_string_to_usb(debug_msg);
  HAL_I2C_EnableListen_IT(hi2c);
}

