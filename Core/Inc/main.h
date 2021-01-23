/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define EXT_2_Pin GPIO_PIN_2
#define EXT_2_GPIO_Port GPIOE
#define EXT_1_Pin GPIO_PIN_3
#define EXT_1_GPIO_Port GPIOE
#define M1_DIR_Pin GPIO_PIN_4
#define M1_DIR_GPIO_Port GPIOE
#define M1_STEP_Pin GPIO_PIN_5
#define M1_STEP_GPIO_Port GPIOE
#define M1_EN_Pin GPIO_PIN_6
#define M1_EN_GPIO_Port GPIOE
#define DEBUG_LED1_Pin GPIO_PIN_8
#define DEBUG_LED1_GPIO_Port GPIOI
#define DEBUG_LED2_Pin GPIO_PIN_13
#define DEBUG_LED2_GPIO_Port GPIOC
#define DEBUG_TX_Pin GPIO_PIN_2
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_3
#define DEBUG_RX_GPIO_Port GPIOA
#define LIMIT_M3_Pin GPIO_PIN_15
#define LIMIT_M3_GPIO_Port GPIOD
#define LIMIT_M2_Pin GPIO_PIN_2
#define LIMIT_M2_GPIO_Port GPIOG
#define LIMIT_M1_Pin GPIO_PIN_3
#define LIMIT_M1_GPIO_Port GPIOG
#define OUT2_Pin GPIO_PIN_5
#define OUT2_GPIO_Port GPIOG
#define OUT1_Pin GPIO_PIN_6
#define OUT1_GPIO_Port GPIOG
#define EXT_20_Pin GPIO_PIN_9
#define EXT_20_GPIO_Port GPIOC
#define EXT_19_Pin GPIO_PIN_8
#define EXT_19_GPIO_Port GPIOA
#define EXT_18_Pin GPIO_PIN_9
#define EXT_18_GPIO_Port GPIOA
#define EXT_17_Pin GPIO_PIN_10
#define EXT_17_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define EXT_16_Pin GPIO_PIN_13
#define EXT_16_GPIO_Port GPIOH
#define EXT_15_Pin GPIO_PIN_14
#define EXT_15_GPIO_Port GPIOH
#define EXT_14_Pin GPIO_PIN_15
#define EXT_14_GPIO_Port GPIOH
#define EXT_13_Pin GPIO_PIN_0
#define EXT_13_GPIO_Port GPIOI
#define EXT_12_Pin GPIO_PIN_1
#define EXT_12_GPIO_Port GPIOI
#define EXT_11_Pin GPIO_PIN_2
#define EXT_11_GPIO_Port GPIOI
#define EXT_10_Pin GPIO_PIN_3
#define EXT_10_GPIO_Port GPIOI
#define EXT_9_Pin GPIO_PIN_15
#define EXT_9_GPIO_Port GPIOA
#define M3_DIR_Pin GPIO_PIN_0
#define M3_DIR_GPIO_Port GPIOD
#define M3_STEP_Pin GPIO_PIN_1
#define M3_STEP_GPIO_Port GPIOD
#define M3_EN_Pin GPIO_PIN_2
#define M3_EN_GPIO_Port GPIOD
#define M2_DIR_Pin GPIO_PIN_3
#define M2_DIR_GPIO_Port GPIOD
#define M2_STEP_Pin GPIO_PIN_4
#define M2_STEP_GPIO_Port GPIOD
#define M2_EN_Pin GPIO_PIN_5
#define M2_EN_GPIO_Port GPIOD
#define EXT8_Pin GPIO_PIN_6
#define EXT8_GPIO_Port GPIOB
#define EXT7_Pin GPIO_PIN_7
#define EXT7_GPIO_Port GPIOB
#define EXT6_Pin GPIO_PIN_4
#define EXT6_GPIO_Port GPIOI
#define EXT5_Pin GPIO_PIN_5
#define EXT5_GPIO_Port GPIOI
#define EXT4_Pin GPIO_PIN_6
#define EXT4_GPIO_Port GPIOI
#define EXT3_Pin GPIO_PIN_7
#define EXT3_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
