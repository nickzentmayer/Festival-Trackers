/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TP_PA2_Pin GPIO_PIN_2
#define TP_PA2_GPIO_Port GPIOA
#define TP_PC4_Pin GPIO_PIN_4
#define TP_PC4_GPIO_Port GPIOC
#define BTN_Pin GPIO_PIN_5
#define BTN_GPIO_Port GPIOC
#define LCD_BL_Pin GPIO_PIN_9
#define LCD_BL_GPIO_Port GPIOE
#define LCD_RST_Pin GPIO_PIN_10
#define LCD_RST_GPIO_Port GPIOE
#define LORA_RST_Pin GPIO_PIN_12
#define LORA_RST_GPIO_Port GPIOB
#define LORA_DIO1_Pin GPIO_PIN_13
#define LORA_DIO1_GPIO_Port GPIOB
#define LORA_BUSY_Pin GPIO_PIN_14
#define LORA_BUSY_GPIO_Port GPIOB
#define INT_MAG_Pin GPIO_PIN_15
#define INT_MAG_GPIO_Port GPIOB
#define XL_INT0_Pin GPIO_PIN_8
#define XL_INT0_GPIO_Port GPIOD
#define XL_INT1_Pin GPIO_PIN_9
#define XL_INT1_GPIO_Port GPIOD
#define LCD_CS_Pin GPIO_PIN_14
#define LCD_CS_GPIO_Port GPIOA
#define LORA_NSS_Pin GPIO_PIN_15
#define LORA_NSS_GPIO_Port GPIOA
#define BATT_STAT_Pin GPIO_PIN_4
#define BATT_STAT_GPIO_Port GPIOD
#define FG_NALERT_Pin GPIO_PIN_5
#define FG_NALERT_GPIO_Port GPIOD
#define GPS_ON_Pin GPIO_PIN_7
#define GPS_ON_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
