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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp.h"
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
uint8_t pi_rx_buf[128];      // 摄像头(USART3) DMA缓存

/* DEBUG：全局变量，方便 Keil Watch（位置模式：PID输出=脉冲数，不是RPM） */
volatile float g_error_x = 0;       // 当前误差（≈ -offset）
volatile float g_pulse_out = 0;    // PID输出脉冲数
volatile float g_pid_raw_out = 0;  // 保留（兼容）

/* 视觉端直接输出居中偏移量(offset)，即误差信号，故 PID 目标值(setpoint)=0 */
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 初始化PID（参数全部引用 pid.c 里的 TUNE_xxx） */
  PID_Init(&ball_pid, TUNE_KP, TUNE_KI, TUNE_KD, 0.0f, CO_MAX, CO_MIN);

  /* 启动 DMA 循环接收摄像头数据（USART3） */
  if (HAL_UART_Receive_DMA(&huart3, pi_rx_buf, DMA_BUF_SIZE) != HAL_OK)
  {
      Error_Handler();
  }
  /* 开启中断接收，收到1字节进入中断回调（USART2：电机响应） */
  HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);

  /* 等待 Emm V5.0 模块自检完成 */
  HAL_Delay(2000);

  /*
   * 上电：当前位置设为0，电机不转
   * （原ball-beam回零代码已注释，居中跟踪不需要机械回零）
   */
  #if 0
  /* 原回零：单圈就近回零（o_mode=0）—— 电机会转，不适合居中跟踪 */
  {
      uint8_t origin_cmd[] = {0x02, 0x9A, 0x00, 0x00, 0x6B};
      HAL_UART_Transmit(&huart2, origin_cmd, 5, 100);
  }
  HAL_Delay(2000);
  #endif
  {
      uint8_t zero_cmd[] = {0x02, 0x0A, 0x6D, 0x6B};  /* func=0x0A: 当前位置清零 */
      HAL_UART_Transmit(&huart2, zero_cmd, 4, 100);
  }
  HAL_Delay(100);

  /* 使能电机 */
  {
      uint8_t en_cmd[] = {0x02, 0xF3, 0xAB, 0x01, 0x00, 0x6B};  /* addr=2, func=0xF3, state=1(使能), snF=0 */
      HAL_UART_Transmit(&huart2, en_cmd, 6, 100);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ===== 视觉跟踪移植：位置式PID + 双增益切换（偏移量域） ===== */

    /* 检查摄像头是否有有效数据（999 = 丢失目标） */
    if (g_latest_error_x != 999)
    {
        /* 直接把居中偏移量(有符号)当误差喂入 PID —— 不要强转 uint16，会毁掉负数 */
        Ball_Control_Once(g_latest_error_x);

        /* 更新调试变量 */
        g_error_x = ball_pid.Error0;
        g_pulse_out = ball_pid.out;
    }

    /* 周期打印调试信息（每200ms） */
    {
        static u32 last_print_tick = 0;
        if (uwTick - last_print_tick >= 200)
        {
            last_print_tick = uwTick;
            char buf[80];
            char *dir = (ball_direction == 0) ? "CW" : "CCW";
            sprintf(buf, "err=%.1f  CO=%.1f  int=%.1f  Kp=%.2f  off=%d  dir=%s\r\n",
                    ball_pid.Error0, ball_pid.out, ball_pid.err_int,
                    ball_pid.Kp, (int)g_latest_error_x, dir);
            uart1_print(buf);
        }
    }

    HAL_Delay(1); // 1ms 主循环节流
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
