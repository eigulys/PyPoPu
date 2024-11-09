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
#include "stm32f4xx_hal.h"

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
#define M1_Pin GPIO_PIN_13
#define M1_GPIO_Port GPIOC
#define M1_EXTI_IRQn EXTI15_10_IRQn
#define ROT4_A_Pin GPIO_PIN_0
#define ROT4_A_GPIO_Port GPIOA
#define ROT4_B_Pin GPIO_PIN_1
#define ROT4_B_GPIO_Port GPIOA
#define ROT2_A_Pin GPIO_PIN_6
#define ROT2_A_GPIO_Port GPIOA
#define ROT2_B_Pin GPIO_PIN_7
#define ROT2_B_GPIO_Port GPIOA
#define ROT1_A_Pin GPIO_PIN_15
#define ROT1_A_GPIO_Port GPIOA
#define M2_Pin GPIO_PIN_12
#define M2_GPIO_Port GPIOC
#define M2_EXTI_IRQn EXTI15_10_IRQn
#define LED_D1_Pin GPIO_PIN_2
#define LED_D1_GPIO_Port GPIOD
#define ROT1_B_Pin GPIO_PIN_3
#define ROT1_B_GPIO_Port GPIOB
#define ROT3_A_Pin GPIO_PIN_6
#define ROT3_A_GPIO_Port GPIOB
#define ROT3_B_Pin GPIO_PIN_7
#define ROT3_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
