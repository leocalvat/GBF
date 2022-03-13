/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SIG3_1_Pin GPIO_PIN_6
#define SIG3_1_GPIO_Port GPIOA
#define SIG3_2_Pin GPIO_PIN_7
#define SIG3_2_GPIO_Port GPIOA
#define SIG2_1_Pin GPIO_PIN_10
#define SIG2_1_GPIO_Port GPIOB
#define SIG2_2_Pin GPIO_PIN_11
#define SIG2_2_GPIO_Port GPIOB
#define SIG1_1N_Pin GPIO_PIN_13
#define SIG1_1N_GPIO_Port GPIOB
#define SIG1_2N_Pin GPIO_PIN_14
#define SIG1_2N_GPIO_Port GPIOB
#define SIG1_1_Pin GPIO_PIN_8
#define SIG1_1_GPIO_Port GPIOA
#define SIG1_2_Pin GPIO_PIN_9
#define SIG1_2_GPIO_Port GPIOA
#define SIG1_3_Pin GPIO_PIN_10
#define SIG1_3_GPIO_Port GPIOA
#define SIG1_4_Pin GPIO_PIN_11
#define SIG1_4_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
