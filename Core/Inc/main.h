/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOE
#define WHEEL1_Pin GPIO_PIN_0
#define WHEEL1_GPIO_Port GPIOB
#define REV_CH1_Pin GPIO_PIN_7
#define REV_CH1_GPIO_Port GPIOE
#define REV_CH2_Pin GPIO_PIN_9
#define REV_CH2_GPIO_Port GPIOE
#define REV_CH3_Pin GPIO_PIN_11
#define REV_CH3_GPIO_Port GPIOE
#define REV_CH4_Pin GPIO_PIN_13
#define REV_CH4_GPIO_Port GPIOE
#define WHEEL2_Pin GPIO_PIN_12
#define WHEEL2_GPIO_Port GPIOD
#define WHEEL3_Pin GPIO_PIN_13
#define WHEEL3_GPIO_Port GPIOD
#define WHEEL4_Pin GPIO_PIN_14
#define WHEEL4_GPIO_Port GPIOD
#define UART_SEND_Pin GPIO_PIN_10
#define UART_SEND_GPIO_Port GPIOC
#define UART_RECV_Pin GPIO_PIN_11
#define UART_RECV_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
