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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "OLED.h"
#include "JY61P.h"
#include "elog.h"
#include "Key.h"
#include "LED.h"
#include "LASER.h"
#include "motor.h"
#include "ENCODER.h"
#include "Timer.h"
#include "Serial.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define JY_WARMUP_MS  2000   /* JY61P 上电自检约1s, 期间输出乱码, 前2s丢弃 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t oledTick = 0;   /* OLED 上次刷新时刻 */
static uint32_t logTick = 0;    /* 串口/蓝牙日志上次打印时刻 */
static JY61P_Driver *jy61p = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void MotorTest(void);
static void JY61P_Test(void);
static void JY61P_ReadAngles(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  电机自检：KEY1 触发。逐个电机正转/反转短时间，OLED + 串口显示进度
 */
static void MotorTest(void)
{
  const int16_t duty = 30;   /* 占空比 30/99 ≈ 30% */
  char line[24];

  elog_i("MOTOR", "=== Motor + Encoder test start ===");
  for (uint8_t m = MOTOR_1; m <= MOTOR_4; m++)
  {
    int32_t enc;
    LED_ON();
    OLED_Clear();
    OLED_ShowString(0, 0, "MotorTest", OLED_8X16);

    /* forward: drive motor then read encoder delta */
    Encoder_ResetCount(m);
    snprintf(line, sizeof(line), "M%d FWD", m);
    OLED_ShowString(0, 16, line, OLED_8X16);
    OLED_Update();
    Set_PWM(m, duty);
    HAL_Delay(600);
    Set_PWM(m, 0);
    enc = Encoder_GetCount(m);
    elog_i("MOTOR", "M%d FWD PWM=%d ENC=%ld", m, duty, (long)enc);
    HAL_Delay(150);

    /* reverse: drive motor then read encoder delta */
    Encoder_ResetCount(m);
    snprintf(line, sizeof(line), "M%d REV", m);
    OLED_ShowString(0, 16, line, OLED_8X16);
    OLED_Update();
    Set_PWM(m, -duty);
    HAL_Delay(600);
    Set_PWM(m, 0);
    enc = Encoder_GetCount(m);
    elog_i("MOTOR", "M%d REV PWM=%d ENC=%ld", m, -duty, (long)enc);
    HAL_Delay(200);
  }
  LED_OFF();
  elog_i("MOTOR", "=== Motor + Encoder test done ===");
  OLED_Clear();
  OLED_ShowString(0, 0, "Test Done", OLED_8X16);
  OLED_Update();
}

/**
 * @brief  JY61P 初始化（软件 I2C: SCL=PB6, SDA=PB7, 地址 0x50）
 */
static void JY61P_Test(void)
{
  jy61p = JY61P_Create(0x50, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7);
  if (jy61p == NULL)
  {
    elog_e("JY", "JY61P create failed");
    return;
  }
  jy61p->fun->Init(jy61p);
  elog_i("JY", "JY61P init OK (addr=0x50, SW I2C PB6/PB7)");

  /* 探测 I2C 设备是否应答 */
  if (JY61P_Check(jy61p))
  {
    elog_i("JY", "I2C detect: OK (device ACK)");
  }
  else
  {
    elog_e("JY", "I2C detect: FAIL (no ACK)");
    elog_e("JY", "Check wiring/power; or increase MYI2C_DELAY_CNT in MyI2C.c");
  }
}

/**
 * @brief  读取 JY61P 三个姿态角到 var
 */
static void JY61P_ReadAngles(void)
{
  if (jy61p != NULL)
  {
    jy61p->fun->ROLL_GET(jy61p);
    jy61p->fun->PITCH_GET(jy61p);
    jy61p->fun->YAW_GET(jy61p);
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  /* ---- Hardware 外设初始化（GPIO/TIM/UART 已由 MX_xxx_Init 完成） ---- */
  OLED_Init();      /* I2C2 OLED */
  LED_Init();       /* PB2 高电平亮 */
  Laser_Init();     /* PC13 激光，默认关 */
  Timer_Init();     /* TIM9 1ms 定时中断，内部驱动 Key_Tick() */
  Key_Init();       /* KEY1=PD3, KEY2=PC12 */
  Motor_Init();     /* TIM8 四路 PWM + 编码器启动 */
  Serial_Init();    /* UART4 通用调试串口(PC10/PC11) */

  /* ---- EasyLogger: 输出到 USART3(PD8/PD9)，可接蓝牙透传到电脑 ---- */
  elog_init();
  ELOG_FMT_TABLE();
  elog_start();
  elog_i("MAIN", "System boot, %lu MHz", (unsigned long)(SystemCoreClock / 1000000U));

  OLED_Clear();
  OLED_ShowString(0, 0, "STM32F407", OLED_8X16);
  OLED_ShowString(0, 16, "Init OK", OLED_8X16);
  OLED_Update();

  /* ---- JY61P 姿态传感器（软件 I2C: SCL=PB6, SDA=PB7） ---- */
  JY61P_Test();
  elog_i("JY", "Warm-up %u ms: discard boot garbage", (unsigned)JY_WARMUP_MS);

  elog_i("MAIN", "KEY1=MotorTest  KEY2=JY61P_YawZero");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LED_ON();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* KEY1 按下 -> 电机自检（仅按键触发，安全） */
    if (Key_Check(KEY_1, KEY_DOWN))
    {
      MotorTest();
    }
    /* KEY2 按下 -> JY61P 航向角归零 */
    if (Key_Check(KEY_2, KEY_DOWN))
    {
      if (jy61p != NULL)
      {
        elog_i("JY", "Yaw zeroing...");
        jy61p->fun->YAW_ZERO(jy61p);
        jy61p->var.yaw = 0;
        elog_i("JY", "Yaw zero done");
      }
    }

    /* 每 100ms 读取姿态角并刷新 OLED */
    if (HAL_GetTick() - oledTick >= 100)
    {
      oledTick = HAL_GetTick();
      /* 上电前 JY_WARMUP_MS 丢弃乱码: 此时 var 保持0, OLED显示0 */
      if (HAL_GetTick() >= JY_WARMUP_MS)
      {
        JY61P_ReadAngles();
      }

      OLED_Clear();
      OLED_ShowString(0, 0, "R:", OLED_6X8);
      OLED_ShowFloatNum(24, 0,  (jy61p ? jy61p->var.roll  : 0), 3, 1, OLED_6X8);
      OLED_ShowString(0, 8,  "P:", OLED_6X8);
      OLED_ShowFloatNum(24, 8,  (jy61p ? jy61p->var.pitch : 0), 3, 1, OLED_6X8);
      OLED_ShowString(0, 16, "Y:", OLED_6X8);
      OLED_ShowFloatNum(24, 16, (jy61p ? jy61p->var.yaw   : 0), 3, 1, OLED_6X8);
      OLED_ShowString(0, 24, "ENC1-4:", OLED_6X8);
      OLED_ShowSignedNum(0,  32, Encoder_GetCount(1), 6, OLED_6X8);
      OLED_ShowSignedNum(48, 32, Encoder_GetCount(2), 6, OLED_6X8);
      OLED_ShowSignedNum(0,  40, Encoder_GetCount(3), 6, OLED_6X8);
      OLED_ShowSignedNum(48, 40, Encoder_GetCount(4), 6, OLED_6X8);
      OLED_ShowString(0, 48, "K1:Motor K2:Zero", OLED_6X8);
      OLED_Update();
    }

    /* 每 500ms 经蓝牙串口(USART3)打印姿态角与编码器计数 */
    if (HAL_GetTick() - logTick >= 500)
    {
      logTick = HAL_GetTick();
      if (jy61p != NULL)
      {
        elog_i("JY", "R=%.1f P=%.1f Y=%.1f",
               jy61p->var.roll, jy61p->var.pitch, jy61p->var.yaw);
      }
      elog_i("ENC", "M1=%ld M2=%ld M3=%ld M4=%ld",
             (long)Encoder_GetCount(1), (long)Encoder_GetCount(2),
             (long)Encoder_GetCount(3), (long)Encoder_GetCount(4));
    }
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
