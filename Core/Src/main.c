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
uint8_t user_last_command;
uint8_t last_moving_method = 'H';
GPIO_PinState SA;
GPIO_PinState SB;
GPIO_PinState SC;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// move forward
void PQ() {
    HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_SET);
    WHEEL1_CCR = DEFAULT_CCR;
    WHEEL2_CCR = DEFAULT_CCR;
    WHEEL3_CCR = DEFAULT_CCR;
    WHEEL4_CCR = DEFAULT_CCR;
}

// moving backward
void PH() {
    HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_RESET);
    WHEEL1_CCR = DEFAULT_CCR;
    WHEEL2_CCR = DEFAULT_CCR;
    WHEEL3_CCR = DEFAULT_CCR;
    WHEEL4_CCR = DEFAULT_CCR;
}

// turn left
void PZ() {
    HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_RESET);
    WHEEL1_CCR = DEFAULT_CCR;
    WHEEL2_CCR = DEFAULT_CCR;
    WHEEL3_CCR = DEFAULT_CCR;
    WHEEL4_CCR = DEFAULT_CCR;
}

// turn right
void PY() {
    HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_RESET);
    WHEEL1_CCR = DEFAULT_CCR;
    WHEEL2_CCR = DEFAULT_CCR;
    WHEEL3_CCR = DEFAULT_CCR;
    WHEEL4_CCR = DEFAULT_CCR;
}

// moving forward with turn left
void PL() {
    HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_SET);
    WHEEL1_CCR = 0;
    WHEEL2_CCR = 0;
    WHEEL3_CCR = DEFAULT_CCR;
    WHEEL4_CCR = DEFAULT_CCR;
}

// moving forward with turn right
void PR() {
    HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_SET);
    WHEEL1_CCR = DEFAULT_CCR;
    WHEEL2_CCR = DEFAULT_CCR;
    WHEEL3_CCR = 0;
    WHEEL4_CCR = 0;
}

// Halt
void Stop() {
    HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_RESET);
    WHEEL1_CCR = 0;
    WHEEL2_CCR = 0;
    WHEEL3_CCR = 0;
    WHEEL4_CCR = 0;
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
  /* USER CODE BEGIN 2 */
    // move 100ms to tell user we boot on
    /*HAL_TIM_PWM_Start(&WHEEL1_htim, WHEEL1_TIM_CHANNEL);
    HAL_Delay(300);
    HAL_TIM_PWM_Stop(&WHEEL1_htim, WHEEL1_TIM_CHANNEL);
    HAL_Delay(300);

    HAL_TIM_PWM_Start(&WHEEL2_htim, WHEEL2_TIM_CHANNEL);
    HAL_Delay(300);
    HAL_TIM_PWM_Stop(&WHEEL2_htim, WHEEL2_TIM_CHANNEL);
    HAL_Delay(300);

    HAL_TIM_PWM_Start(&WHEEL3_htim, WHEEL3_TIM_CHANNEL);
    HAL_Delay(300);
    HAL_TIM_PWM_Stop(&WHEEL3_htim, WHEEL3_TIM_CHANNEL);
    HAL_Delay(300);

    HAL_TIM_PWM_Start(&WHEEL4_htim, WHEEL4_TIM_CHANNEL);
    HAL_Delay(300);
    HAL_TIM_PWM_Stop(&WHEEL4_htim, WHEEL4_TIM_CHANNEL);
    HAL_Delay(300);*/

    //  Enable UART serial port idle interrupt
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    //  Enable DMA reception to Idle
    //  "rx_data_length" is defined in "usart.h"
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_data, rx_data_length);
    HAL_Delay(300);
    last_moving_method = 'A';

    HAL_TIM_PWM_Start(&WHEEL1_htim, WHEEL1_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&WHEEL2_htim, WHEEL2_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&WHEEL3_htim, WHEEL3_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&WHEEL4_htim, WHEEL4_TIM_CHANNEL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        if (last_moving_method == 'A') {
            SA = HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin);
            SB = HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin);
            SC = HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin);
            //[][*][]
            if (SA == GPIO_PIN_RESET && SB == GPIO_PIN_SET && SC == GPIO_PIN_RESET) {
                PQ();
            }
            // [*][][]
            else if (SA == GPIO_PIN_SET && SB == GPIO_PIN_RESET && SC == GPIO_PIN_RESET) {
                PZ();
            }
            // [][][*]
            else if (SA == GPIO_PIN_RESET && SB == GPIO_PIN_RESET && SC == GPIO_PIN_SET) {
                PY();
            }
            // [*][][*]
            else if (SA == GPIO_PIN_SET && SB == GPIO_PIN_RESET && SC == GPIO_PIN_SET) {
                PZ();
            }
            // [][*][*]
            else if (SA == GPIO_PIN_RESET && SB == GPIO_PIN_SET && SC == GPIO_PIN_SET) {
                PR();
            }
            // [*][*][]
            else if (SA == GPIO_PIN_SET && SB == GPIO_PIN_SET && SC == GPIO_PIN_RESET) {
                PL();
            }
            // [*][*][*]
            else if (SA == GPIO_PIN_RESET && SB == GPIO_PIN_RESET && SC == GPIO_PIN_RESET) {
                PZ();
            } else {
                Stop();
            }
        } else {
            HAL_Delay(1000);
        }

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
    // toggle LED2 to indicate that we have received a command.
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    switch (rx_data[0]) {
        //  'M': Continuous motion
        case '0':
        case '1': {
            user_last_command = 'M';
            // switch moving method
            if (last_moving_method == 'H') {
                // init PWM
                HAL_TIM_PWM_Start(&WHEEL1_htim, WHEEL1_TIM_CHANNEL);
                HAL_TIM_PWM_Start(&WHEEL2_htim, WHEEL2_TIM_CHANNEL);
                HAL_TIM_PWM_Start(&WHEEL3_htim, WHEEL3_TIM_CHANNEL);
                HAL_TIM_PWM_Start(&WHEEL4_htim, WHEEL4_TIM_CHANNEL);
            }
            // Direction of wheel 1
            if (rx_data[0] == '0') {
                HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH1_GPIO_Port, REV_CH1_Pin, GPIO_PIN_SET);
            }
            // Direction of wheel 2
            if (rx_data[1] == '0') {
                HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH2_GPIO_Port, REV_CH2_Pin, GPIO_PIN_SET);
            }
            // Direction of wheel 3
            if (rx_data[2] == '0') {
                HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH3_GPIO_Port, REV_CH3_Pin, GPIO_PIN_SET);
            }
            // Direction of wheel 4
            if (rx_data[3] == '0') {
                HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(REV_CH4_GPIO_Port, REV_CH4_Pin, GPIO_PIN_SET);
            }

            // Speed of wheel 1
            HAL_TIM_PWM_Stop(&WHEEL1_htim, WHEEL1_TIM_CHANNEL);
            WHEEL1_CCR = (rx_data[4] - '0') * 64;
            HAL_TIM_PWM_Start(&WHEEL1_htim, WHEEL1_TIM_CHANNEL);

            // Speed of wheel 2
            HAL_TIM_PWM_Stop(&WHEEL2_htim, WHEEL2_TIM_CHANNEL);
            WHEEL2_CCR = (rx_data[5] - '0') * 64;
            HAL_TIM_PWM_Start(&WHEEL2_htim, WHEEL2_TIM_CHANNEL);

            // Speed of wheel 3
            HAL_TIM_PWM_Stop(&WHEEL3_htim, WHEEL3_TIM_CHANNEL);
            WHEEL3_CCR = (rx_data[6] - '0') * 64;
            HAL_TIM_PWM_Start(&WHEEL3_htim, WHEEL3_TIM_CHANNEL);

            // Speed of wheel 4
            HAL_TIM_PWM_Stop(&WHEEL4_htim, WHEEL4_TIM_CHANNEL);
            WHEEL4_CCR = (rx_data[7] - '0') * 64;
            HAL_TIM_PWM_Start(&WHEEL4_htim, WHEEL4_TIM_CHANNEL);

            // Set last_moving_method to 'M'
            last_moving_method = 'M';
            break;
        }
            // 'E': Echo that we are online
        case 'E': {

            // erase command to avoid calling echo again
            rx_data[0] = '0';
            tx_data[0] = user_last_command;
            for (int i = 1; i < tx_data_length; ++i) {
                tx_data[i] = rx_data[i];
            }
            __HAL_UART_DISABLE_IT(&huart4, UART_IT_IDLE);
            // send buffer, reused rx_data as tx_data
            // Notice! calling this function may trigger a UART interrupt
            // which will call MyUartCallbackHandler() again
            HAL_UART_Transmit(&huart4, tx_data, tx_data_length, 300);
            __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
            user_last_command = 'E';
            break;
        }
        // 'P': Packaged commands
        case 'P': {
            // switch moving method
            if (last_moving_method == 'H') {
                // init PWM
                HAL_TIM_PWM_Start(&WHEEL1_htim, WHEEL1_TIM_CHANNEL);
                HAL_TIM_PWM_Start(&WHEEL2_htim, WHEEL2_TIM_CHANNEL);
                HAL_TIM_PWM_Start(&WHEEL3_htim, WHEEL3_TIM_CHANNEL);
                HAL_TIM_PWM_Start(&WHEEL4_htim, WHEEL4_TIM_CHANNEL);
            }
            switch (rx_data[1]) {
                // moving forward
                case 'Q': {
                    PQ();
                    break;
                }
                // moving backward
                case 'H': {
                    PH();
                    HAL_TIM_PWM_Stop(&WHEEL1_htim, TIM_CHANNEL_1);
                    HAL_TIM_PWM_Stop(&WHEEL2_htim, TIM_CHANNEL_2);
                    HAL_TIM_PWM_Stop(&WHEEL3_htim, TIM_CHANNEL_3);
                    HAL_TIM_PWM_Stop(&WHEEL4_htim, TIM_CHANNEL_4);
                    user_last_command = 'H';
                    last_moving_method = 'H';
                    break;
                }
                // turn left
                case 'Z': {
                    PZ();
                    break;
                }
                // turn right
                case 'Y': {
                    PY();
                    break;
                }
                // moving forward with turn left
                case 'L': {
                    PL();
                    break;
                }
                // moving forward with turn right
                case 'R': {
                    PR();
                    break;
                }
                default:
                    break;
            }
            user_last_command = 'P';
            last_moving_method = 'M';
            break;
        }
        // 'H': Halt
        case 'H': {
            Stop();
            break;
        }
        // 'A': Follow the line
        case 'A': {
            if (last_moving_method == 'H') {
                HAL_TIM_PWM_Start(&WHEEL1_htim, WHEEL1_TIM_CHANNEL);
                HAL_TIM_PWM_Start(&WHEEL2_htim, WHEEL2_TIM_CHANNEL);
                HAL_TIM_PWM_Start(&WHEEL3_htim, WHEEL3_TIM_CHANNEL);
                HAL_TIM_PWM_Start(&WHEEL4_htim, WHEEL4_TIM_CHANNEL);
            }
            user_last_command = 'A';
            last_moving_method = 'A';
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
    if (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_RX) {
        HAL_UART_AbortReceive(huart);
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
    // Turn on LED2
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
