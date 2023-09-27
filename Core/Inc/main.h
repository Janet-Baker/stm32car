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
#define SENSOR1_Pin GPIO_PIN_3
#define SENSOR1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOE
#define SENSOR2_Pin GPIO_PIN_3
#define SENSOR2_GPIO_Port GPIOC
#define SENSOR3_Pin GPIO_PIN_1
#define SENSOR3_GPIO_Port GPIOA
#define WHEEL1_Pin GPIO_PIN_6
#define WHEEL1_GPIO_Port GPIOA
#define WHEEL2_Pin GPIO_PIN_7
#define WHEEL2_GPIO_Port GPIOA
#define WHEEL3_Pin GPIO_PIN_0
#define WHEEL3_GPIO_Port GPIOB
#define WHEEL4_Pin GPIO_PIN_1
#define WHEEL4_GPIO_Port GPIOB
#define REV_CH1_Pin GPIO_PIN_7
#define REV_CH1_GPIO_Port GPIOE
#define REV_CH2_Pin GPIO_PIN_9
#define REV_CH2_GPIO_Port GPIOE
#define REV_CH3_Pin GPIO_PIN_11
#define REV_CH3_GPIO_Port GPIOE
#define REV_CH4_Pin GPIO_PIN_13
#define REV_CH4_GPIO_Port GPIOE
#define UART_SEND_Pin GPIO_PIN_10
#define UART_SEND_GPIO_Port GPIOC
#define UART_RECV_Pin GPIO_PIN_11
#define UART_RECV_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define DEFAULT_CCR 575
#define DEFAULT_ARR 1151
#define WHEEL1_htim htim3
#define WHEEL1_TIM_CHANNEL TIM_CHANNEL_1
#define WHEEL1_CCR WHEEL1_htim.Instance->CCR1
#define WHEEL2_htim htim3
#define WHEEL2_TIM_CHANNEL TIM_CHANNEL_2
#define WHEEL2_CCR WHEEL2_htim.Instance->CCR2
#define WHEEL3_htim htim3
#define WHEEL3_TIM_CHANNEL TIM_CHANNEL_3
#define WHEEL3_CCR WHEEL3_htim.Instance->CCR3
#define WHEEL4_htim htim3
#define WHEEL4_TIM_CHANNEL TIM_CHANNEL_4
#define WHEEL4_CCR WHEEL4_htim.Instance->CCR4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
