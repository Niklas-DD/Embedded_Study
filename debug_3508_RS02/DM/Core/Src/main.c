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
#include "bsp_can.h"
#include "RS02.h"
#include "Dji3508.h"
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

// RS02电机配置参数
#define RS02_CAN_ID 0x01U            // RS02电机CAN ID
#define RS02_MASTER_ID 0xFDU         // 主控CAN ID
#define RS02_TARGET_SPEED_RAD_S 1.0f // 目标速度（弧度/秒）
#define RS02_LIMIT_CURRENT_A 2.0f    // 限制电流（安培）
#define RS02_CMD_PERIOD_MS 10U       // 控制命令发送周期（毫秒）

RobStride_Motor motor1;                 // RS02电机实例
static uint32_t rs02_last_cmd_tick = 0; // 上次发送命令的时间戳
static uint8_t rs02_inited = 0;         // RS02初始化标志位

extern CAN_HandleTypeDef hcan1; // 外部声明CAN句柄
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// RS02相关函数声明
static void RS02_UserInit(void); // RS02用户初始化函数
static void RS02_Task(void);     // RS02任务处理函数
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief RS02电机用户初始化函数
 * @note 配置CAN通信、设置电机参数并使能电机
 */
static void RS02_UserInit(void)
{
  bsp_can_init(); // 初始化CAN底层驱动

  RobStride_Motor_init(&motor1, RS02_CAN_ID, false); // 初始化RS02电机对象，ID=1，非MIT模式
  motor1.Master_CAN_ID = RS02_MASTER_ID;             // 设置主控CAN ID
  Disenable_Motor(&motor1, 1);                       // 禁用电机（确保初始状态安全）
  HAL_Delay(20);                                     // 延时等待

  Get_RobStride_Motor_parameter(&motor1, 0x7005); // 读取当前控制模式参数
  HAL_Delay(20);                                  // 延时等待响应

  Set_RobStride_Motor_parameter(&motor1, 0x7005, Speed_control_mode, Set_mode); // 设置为速度控制模式
  HAL_Delay(5);                                                                 // 延时等待

  Set_RobStride_Motor_parameter(&motor1, 0x7018, RS02_LIMIT_CURRENT_A, Set_parameter); // 设置电流限制为2A
  HAL_Delay(5);                                                                        // 延时等待

  Enable_Motor(&motor1); // 使能电机
  HAL_Delay(20);         // 延时等待电机就绪

  rs02_last_cmd_tick = HAL_GetTick(); // 记录初始时间戳
  rs02_inited = 1;                    // 标记初始化完成
}

/**
 * @brief RS02电机周期性任务函数
 * @note 按照设定周期发送速度控制指令
 */
static void RS02_Task(void)
{
  if (!rs02_inited) // 检查是否已完成初始化
  {
    return; // 未初始化则直接返回
  }

  // 检查是否到达控制周期
  if ((HAL_GetTick() - rs02_last_cmd_tick) >= RS02_CMD_PERIOD_MS)
  {
    rs02_last_cmd_tick = HAL_GetTick(); // 更新时间戳
    // 发送速度控制指令：目标速度1rad/s，电流限制2A
    RobStride_Motor_Speed_control(&motor1, RS02_TARGET_SPEED_RAD_S, RS02_LIMIT_CURRENT_A);
  }
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  RS02_UserInit(); // 执行RS02电机初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    RS02_Task();                    // 执行RS02电机控制任务
    send_chassis_cur1_4(0, 800, 0); // 发送3508电机电流控制指令（ID1:0A, ID2:800mA, ID3:0A）
    HAL_Delay(1);                   // 主循环延时1ms

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
/**
 * @brief CAN接收FIFO0消息 pending回调函数
 * @param hcan: CAN句柄指针
 * @note 用于接收并解析RS02电机反馈数据
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader; // CAN接收消息头
  uint8_t rxData[8];            // CAN接收数据缓冲区

  if (hcan->Instance == CAN1) // 确认是CAN1外设
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData); // 从FIFO0读取CAN消息

    // 根据帧类型选择正确的ID进行解析
    if (rxHeader.IDE == CAN_ID_EXT)
    {
      RobStride_Motor_Analysis(&motor1, rxData, rxHeader.ExtId); // 解析扩展帧ID的电机数据
    }
    else
    {
      RobStride_Motor_Analysis(&motor1, rxData, rxHeader.StdId); // 解析标准帧ID的电机数据
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
