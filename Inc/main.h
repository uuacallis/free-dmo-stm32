/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  * This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// Functions from main.c that we need to make "public" for other files to call
void FLASH_WriteDefaultSKUIndex(uint32_t index);
void send_prompt(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// These definitions were removed by the CubeMX migration. Adding them back
// fixes the build errors in stm32f1xx_hal_msp.c and stm32f1xx_it.c
#define EXTI0_IN_PWDN_Pin GPIO_PIN_0
#define EXTI0_IN_PWDN_GPIO_Port GPIOA
#define I2C2_SCL_READER_Pin GPIO_PIN_10
#define I2C2_SCL_READER_GPIO_Port GPIOB
#define I2C2_SDA_READER_Pin GPIO_PIN_11
#define I2C2_SDA_READER_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

