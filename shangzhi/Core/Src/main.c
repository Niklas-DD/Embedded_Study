/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Dji3508.h"
#include "RS02.h"
#include "DRV8860.h"
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
// DJI 3508:ID=1 ?? moto_chassis[0],ID=2 ?? moto_chassis[1]
RobStride_Motor motor1;
Motor3LoopCtrl_t motor1_3loop;
Motor3LoopCtrl_t motor2_3loop;

extern CAN_HandleTypeDef hcan1; // ????CAN??
uint8_t rxData[8] = {0};

// 3508 ????????????????????? 90�??
volatile uint8_t m3508_feedback_ok[4] = {0};
uint8_t m3508_target_set = 0;
float m3508_target1 = 0.0f;
float m3508_target2 = 0.0f;

// 3508 ????? 8192,get_total_angle() ???? 3508 ???,
// ????? 2048 ????? 90�:8192 * 90 / 360 = 2048
#define M3508_OUTPUT_90DEG_CNT (2048.0f)
#define M3508_CTRL_PERIOD_MS (10U)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// RobStride_Motor motor1;
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  RS02_UserInit();
  Motor3Loop_Init(&motor1_3loop);
  Motor3Loop_Init(&motor2_3loop);
  DRV8860_init(&drv_1);
  DRV8860_init(&drv_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    channel_set(&drv_1, DRV8860_CHANNEL_1, 1);
    HAL_Delay(1000);
    channel_set(&drv_1, DRV8860_CHANNEL_1, 0);
    // channel_set(&drv_2, DRV8860_CHANNEL_5, 1);
    RS02_Task();
    if ((m3508_feedback_ok[0] != 0) && (m3508_feedback_ok[1] != 0))
    {
      if (m3508_target_set == 0)
      {
        m3508_target1 = moto_chassis[0].total_angle_output + M3508_OUTPUT_90DEG_CNT;
        m3508_target2 = moto_chassis[1].total_angle_output + M3508_OUTPUT_90DEG_CNT;
        motor1_3loop.target_pos = m3508_target1;
        motor2_3loop.target_pos = m3508_target2;
        m3508_target_set = 1;
      }

      // speed_mode = 0:????????????? C620 ?????
      int16_t cur_cmd1 = Motor3Loop_Update(&motor1_3loop, &moto_chassis[0], 0);
      int16_t cur_cmd2 = Motor3Loop_Update(&motor2_3loop, &moto_chassis[1], 0);
      send_chassis_cur1_4(cur_cmd1, cur_cmd2, 0);
    }
    else
    {
      send_chassis_cur1_4(0, 0, 0);
    }

    HAL_Delay(M3508_CTRL_PERIOD_MS);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
///**
// * @brief  CAN ?? FIFO0 ?? pending ??
// * @param  hcan: ?? CAN ????
// * @note   ???? RS02 ??? DJI 3508 ?????
// */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader; // ????
  // ??? CAN1 ????
  if (hcan->Instance != CAN1)
  {
    return;
  }

  // ? FIFO0 ???? CAN ??
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
  {
    return;
  }

  if (rxHeader.IDE == CAN_ID_EXT)
  {
    // ------------------------------
    // RS02 / RobStride ????:???
    // ?? ID_ExtId ??????
    // ------------------------------
    RobStride_Motor_Analysis(&motor1, rxData, rxHeader.ExtId);
    //    RS02_CAN_Analysis_C(rxData, rxHeader.ExtId);
  }
  else if (rxHeader.IDE == CAN_ID_STD)
  {
    // ------------------------------
    // DJI 3508 ????:??? 0x201~0x204
    // ???????????????
    // ------------------------------
    switch (rxHeader.StdId)
    {
    case CAN_2006_M1_ID:
    case CAN_2006_M2_ID:
    case CAN_2006_M3_ID:
    case CAN_2006_M4_ID:
    {
      // ?? StdId ??????
      uint8_t i = rxHeader.StdId - CAN_2006_M1_ID;

      get_motor_measure(&moto_chassis[i], rxData);
      get_total_angle(&moto_chassis[i]);

      // ??? 3508 ??????????
      m3508_feedback_ok[i] = 1;
      break;
    }
      // default:
      //   RobStride_Motor_Analysis(&motor1, rxData, rxHeader.StdId);
      //   break;
    }
  }
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
