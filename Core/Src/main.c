/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "M3508_Motor.h"
#include "stdio.h"
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
CAN_RxHeaderTypeDef rx_header;
CAN_TxHeaderTypeDef tx_header = {
    .StdId = 0x200,
    .ExtId = 0,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = ENABLE
};
CAN_FilterTypeDef can_filter = {
    .FilterActivation = ENABLE,
    .FilterBank = 0,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterIdHigh = 0x0000,
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000,
    .FilterMaskIdLow = 0x0000,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT
};
uint8_t rx_data[8];
uint8_t tx_data[8] = {0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint32_t can_tx_mailbox;

float target_angle = 10.0f;
uint8_t stop_flag = 1;

uint8_t step_test_enabled = 0;
uint32_t step_test_start_time = 0;
float step_initial_angle = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void M3508_Motor_RxCallback(const uint8_t rx_data[8]);

void Key_Process(void)
{
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(40);  // 消抖
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
        {
            stop_flag = !stop_flag;

            if (stop_flag == 1)
            {
                M3508_Motor_Stop();
            }

            while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET);
        }
    }
}

void Step_Response_Test(void)
{
    static uint8_t last_stop_flag = 1;

    if (stop_flag == 0 && last_stop_flag == 1)
    {
        step_test_start_time = HAL_GetTick();
        step_initial_angle = target_angle;
        step_test_enabled = 1;
    }

    if (stop_flag == 0 && step_test_enabled)
    {
        uint32_t current_time = HAL_GetTick();
        if (current_time - step_test_start_time > 2000)
        {
            target_angle = step_initial_angle + 10.0f;
            step_test_enabled = 0;

            printf("Step test: %.1f -> %.1f deg\n", step_initial_angle, target_angle);
        }
    }

    last_stop_flag = stop_flag;
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
    target_angle = 10.0f;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART8_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_TIM_Base_Start_IT(&htim6);

  float position_kp = 10.0f;
  float position_ki = 0.5f;
  float position_kd = 0.1f;

  float speed_kp = 5.0f;
  float speed_ki = 0.3f;
  float speed_kd = 0.05f;

  M3508_Motor_SetPID(position_kp, position_ki, position_kd, speed_kp, speed_ki, speed_kd);


  M3508_Motor_SetPositionSpeedMode();

  M3508_Motor_Stop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      Key_Process();
      Step_Response_Test();

      HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
