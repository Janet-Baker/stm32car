/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Convert the input speed to make it more intuitive
uint16_t logarithmic_mapping(uint16_t x){
    uint16_t min_x = 0;
    uint16_t max_x = 999;
    uint16_t min_y = 999;
    uint16_t max_y = 99;
    uint16_t y = min_y + (max_y - min_y) * log10(1 + (x - min_x) / (max_x - min_x));
    return y;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  // Report that we have initialized.
  for(int i=0; HAL_UART_GetState(&huart4) != HAL_UART_STATE_READY; ++i){
      if (i > 100) {
          Error_Handler();
      }
      HAL_Delay(100);
  }
  uint8_t stateOk[] = "BOOT_OK";
  HAL_UART_Transmit_DMA(&huart4,stateOk, 7);

//  Launch completed and ready to receive data
    for (int i = 0; HAL_UART_GetState(&huart4) != HAL_UART_STATE_READY; ++i) {
        HAL_Delay(100);
    }
//  Enable serial port idle interrupt
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//  Enable DMA reception to Idle
//  "rx_data_length" is defined in "usart.h"
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_data, rx_data_length);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
}

/* USER CODE BEGIN 4 */
// Pending Implementation
void MyUartCallbackHandler(void) {
//    Detect and execute control command
//    TO-DO: Fix PWM generation for motor
//    https://zhuanlan.zhihu.com/p/506458493
    switch (rx_data[0]) {
        //  'M': Human-friendly speed control command
        case 'M': {
            // DEBUG, toggle LED2 to indicate that we have received a command.
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            // Direction of wheel 1
            if (rx_data[1] == '0') {
                HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_SET);
            }
            // Direction of wheel 2
            if (rx_data[2] == '0') {
                HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_SET);
            }
            // Direction of wheel 3
            if (rx_data[3] == '0') {
                HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_SET);
            }
            // Direction of wheel 4
            if (rx_data[4] == '0') {
                HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_SET);
            }
            // Speed of wheel 1
            uint16_t pwm1 = logarithmic_mapping(
                    (rx_data[5] - '0') * 100 + (rx_data[6] - '0') * 10 + (rx_data[7] - '0'));
            __HAL_TIM_SET_AUTORELOAD(&htim2, pwm1);
            if (pwm1 < 100 || pwm1 > 1000) {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            } else {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm1 / 2);
            }
            // Speed of wheel 2
            uint16_t pwm2 = logarithmic_mapping(
                    (rx_data[8] - '0') * 100 + (rx_data[9] - '0') * 10 + (rx_data[10] - '0'));
            __HAL_TIM_SET_AUTORELOAD(&htim3, pwm2);
            if (pwm2 < 100 || pwm2 > 1000) {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            } else {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm2 / 2);
            }
            // Speed of wheel 3
            uint16_t pwm3 = logarithmic_mapping(
                    (rx_data[11] - '0') * 100 + (rx_data[12] - '0') * 10 + (rx_data[13] - '0'));
            __HAL_TIM_SET_AUTORELOAD(&htim4, pwm3);
            if (pwm3 < 100 || pwm3 > 1000) {
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
            } else {
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm3 / 2);
            }
            // Speed of wheel 4
            uint16_t pwm4 = logarithmic_mapping(
                    (rx_data[14] - '0') * 100 + (rx_data[15] - '0') * 10 + (rx_data[16] - '0'));
            __HAL_TIM_SET_AUTORELOAD(&htim5, pwm4);
            if (pwm4 < 100 || pwm4 > 1000) {
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
            } else {
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm4 / 2);
            }
            break;
        }
            // 'S': DEBUG usage, raw speed control
        case 'S': {
            // toggle LED2 to indicate that we have received a command.
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            // Direction of wheel 1
            if (rx_data[1] == '0') {
                HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_SET);
            }
            // Direction of wheel 2
            if (rx_data[2] == '0') {
                HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_SET);
            }
            // Direction of wheel 3
            if (rx_data[3] == '0') {
                HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_SET);
            }
            // Direction of wheel 4
            if (rx_data[4] == '0') {
                HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_SET);
            }
            // Speed of wheel 1
            uint16_t pwm1 = (rx_data[5] - '0') * 100 + (rx_data[6] - '0') * 10 + (rx_data[7] - '0');
            __HAL_TIM_SET_AUTORELOAD(&htim2, pwm1);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm1 / 2);

            // Speed of wheel 2
            uint16_t pwm2 = (rx_data[8] - '0') * 100 + (rx_data[9] - '0') * 10 + (rx_data[10] - '0');
            __HAL_TIM_SET_AUTORELOAD(&htim3, pwm2);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm2 / 2);

            // Speed of wheel 3
            uint16_t pwm3 = (rx_data[11] - '0') * 100 + (rx_data[12] - '0') * 10 + (rx_data[13] - '0');
            __HAL_TIM_SET_AUTORELOAD(&htim4, pwm3);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm3 / 2);

            // Speed of wheel 4
            uint16_t pwm4 = (rx_data[14] - '0') * 100 + (rx_data[15] - '0') * 10 + (rx_data[16] - '0');
            __HAL_TIM_SET_AUTORELOAD(&htim5, pwm4);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm4 / 2);
            break;
        }
            // 'E': Echo that we are online
        case 'E': {
            // toggle LED2 to indicate that we have received a command.
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            // erase command to avoid calling echo again
            rx_data[0] = '0';
            __HAL_UART_DISABLE_IT(&huart4, UART_IT_IDLE);
            // send buffer
            // Notice! calling this function may trigger a UART interrupt
            // which will call MyUartCallbackHandler() again
            HAL_UART_Transmit(&huart4, rx_data + 1, tx_data_length, 300);
            __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
            break;
        }
        default:
            return;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    UNUSED(Size);
    // Calling self written interrupt handling functions
    MyUartCallbackHandler();
    // Restart DMA transmission
    // the usage of "HAL_UART_Transmit_DMA" will cause "HAL_BUSY" error
    if (HAL_DMA_GetState(huart->hdmarx) == HAL_DMA_STATE_BUSY) {
        HAL_DMA_Abort(huart->hdmarx);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart,rx_data,rx_data_length);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  // Turn off LED2
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  __disable_irq();
  HAL_NVIC_SystemReset();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
