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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
    // init PWM
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    // Report that we have initialized.
    for (int i = 0; HAL_UART_GetState(&huart4) != HAL_UART_STATE_READY; ++i) {
        if (i > 100) {
            Error_Handler();
        }
        HAL_Delay(100);
    }
    // "BOOT_OK"
    tx_data[0] = 'B'; tx_data[1]='O'; tx_data[2]='O'; tx_data[3]='T'; tx_data[4]='_'; tx_data[5]='O'; tx_data[6]='K';
    HAL_UART_Transmit_DMA(&huart4, tx_data, 7);

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

            // Speed of wheel 2

            // Speed of wheel 3

            // Speed of wheel 4
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

            // Speed of wheel 2

            // Speed of wheel 3

            // Speed of wheel 4
            break;
        }
            // 'E': Echo that we are online
        case 'E': {
            // toggle LED2 to indicate that we have received a command.
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            // erase command to avoid calling echo again
            rx_data[0] = '0';
            __HAL_UART_DISABLE_IT(&huart4, UART_IT_IDLE);
            // send buffer, reused rx_data as tx_data
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
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_data, rx_data_length);
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
    while (1) {
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
