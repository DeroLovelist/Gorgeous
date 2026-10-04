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
#include "Chassis.h"
#include "ServoArm.h"
#include "MissionControl.h"
#include "ArmUartTest.h"   /* 机械臂串口(USART2)收发自检, 见文件内 ARM_UART_TEST_MODE */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* JY61P 陀螺仪上电收敛时间(ms): 冷启动需数秒内部收敛(否则首次上电 roll 有
 * 大→小漂移, 且 yaw 不可靠)。此时间内: 程序不启动任务(按 KEY1 会提示等待),
 * 姿态角也不更新到 OLED。
 * 取值/推荐: 3000~6000(默认 4000)。
 * 影响: 设太小→陀螺仪没稳就跑, 每次上电跑法不一致;
 *       设太大→上电后要多等一会才能开始比赛。若更换姿态传感器型号可再调。 */
#define JY_WARMUP_MS  4000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t oledTick = 0;   /* OLED 上次刷新时刻 */
#if !ARM_UART_TEST_MODE
static uint32_t imuTick = 0;    /* 航向角(yaw)快速读取时刻 */
/* 调试模式(视觉/机械臂)下屏蔽周期性状态日志, 免得淹没收发日志 */
#if !MISSION_DEBUG_VISION_TASK && !MISSION_DEBUG_ARM_SEQ
static uint32_t logTick = 0;    /* 串口/蓝牙日志上次打印时刻 */
static uint32_t dbgTick = 0;    /* 底盘调试日志上次打印时刻 */
static bool     demoRun = false;/* 底盘演示是否运行 (KEY1 切换) */
#endif
#endif
static JY61P_Driver *jy61p = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void MotorTest(void);
static void JY61P_Test(void);
static void JY61P_ReadAngles(void);
static void JY61P_ReadYaw(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  电机自检：逐个电机正转/反转短时间，OLED + 串口显示进度
 * @note   当前已注释调用(KEY1 改为底盘演示), 保留函数备标定使用
 */
static void __attribute__((unused)) MotorTest(void)
{
  const int16_t duty = 30;   /* 占空比 30/99 ≈ 30% */
  char line[24];

  elog_i("MOTOR", "=== Motor + Encoder test start ===");
  Chassis_Suspend();   /* 挂起底盘闭环, 避免覆盖手动 PWM / 编码器计数跳变 */
  for (uint8_t m = MOTOR_1; m <= MOTOR_4; m++)
  {
    int32_t enc;
    LED_ON();
    OLED_Clear();
    OLED_ShowString(0, 0, "MotorTest", OLED_8X16);

    /* forward向前: drive motor then read encoder delta */
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

    /* reverse反转: drive motor then read encoder delta */
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
  Chassis_Resume();   /* 恢复底盘闭环(重新同步编码器并清零位置) */
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
static void __attribute__((unused)) JY61P_ReadAngles(void)
{
  if (jy61p != NULL)
  {
    jy61p->fun->ROLL_GET(jy61p);
    jy61p->fun->PITCH_GET(jy61p);
    jy61p->fun->YAW_GET(jy61p);
  }
}

/**
 * @brief  仅读取 JY61P 航向角 yaw (底盘转向闭环用, 高频调用)
 */
static void __attribute__((unused)) JY61P_ReadYaw(void)
{
  if (jy61p != NULL)
  {
    jy61p->fun->YAW_GET(jy61p);
  }
}

#if !ARM_UART_TEST_MODE
/**
 * @brief  机械臂动作阻塞等待期间的回调: 刷新陀螺仪 yaw
 * @note   由 Mission_SetYawPollHook() 注入, 供 MissionControl.c 的
 *         Mission_Coop_Wait() 在摆臂的几秒里周期性调用。
 *         不刷的话: TIM9 里的航向/转向闭环一直读到冻结的角度 →
 *         原地转向的 s_turn_remaining 永远减不下去 → 车会一直自转。
 */
static void ArmWait_PollYaw(void)
{
  if (HAL_GetTick() >= JY_WARMUP_MS)
  {
    JY61P_ReadYaw();
  }
}
#endif /* !ARM_UART_TEST_MODE */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ---- Hardware 外设初始化（GPIO/TIM/UART 已由 MX_xxx_Init 完成） ---- */
  OLED_Init();      /* I2C2 OLED */
  LED_Init();       /* PB2 高电平亮 */
  Laser_Init();     /* PC13 激光，默认关 */
  Timer_Init();     /* TIM9 1ms 定时中断，内部驱动 Key_Tick() */
  Key_Init();       /* KEY1=PD3, KEY2=PC12 */
  Motor_Init();     /* TIM8 四路 PWM + 编码器启动 */
  Serial_Init();    /* UART4 与 K230 通信串口(PC10/PC11) */
#if ARM_UART_TEST_MODE
  /* ===== 机械臂串口自检模式: 小车(底盘 + 任务状态机)全部不启用 =====
   *  - 不调用 Chassis_Init()  : 底盘闭环不运行 → 电机不会有任何输出
   *  - 不调用 Mission_Init()  : 不会发出任何走位/抓取指令
   *  - 不调用 ServoArm_Init() : 上电不会让机械臂回初始姿态(防误动/夹手)
   *  - 保留 Motor_Init()      : 只为把 4 路方向脚钳在低电平
   *                             (引脚浮空反而可能被干扰误触发) */
  for (uint8_t m = MOTOR_1; m <= MOTOR_4; m++)
  {
    Set_PWM(m, 0);           /* 占空比 0: 电机不转, 仅钳住方向脚 */
  }
  /* 舵机串口自检的初始化(Uart_Init/打印提示/刷屏)放在下面 elog 初始化之后 */
#else
#if MISSION_TEST_NO_ARM
  /* 测试阶段: 跳过机械臂初始化(不动舵机) */
#else
  ServoArm_Init();  /* 飞特舵机机械臂(USART2 直连官方驱动板) */
#endif
  Mission_Init();   /* 任务状态机: 回初始姿态 */
#endif

  /* ---- EasyLogger: 输出到 USART3(PD8/PD9)，可接蓝牙透传到电脑 ---- */
  //轻量日志前置初始化
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

#if ARM_UART_TEST_MODE
  /* 自检模式: 底盘闭环/航向闭环/任务状态机均不初始化(小车不会动) */
  elog_i("MAIN", "ARM UART TEST MODE: car motion code disabled");
  ArmUartTest_Init();        /* 舵机串口 USART2 收发自检 */
#else
  /* ---- 底盘运动控制（麦克纳姆轮, 位置闭环） ---- */
  if (jy61p != NULL)
  {
    Chassis_SetYawSource(&jy61p->var.yaw);   /* 注入航向角数据源(转向闭环用) */
  }
  Chassis_Init();
  /* ⭐ 底盘最大平移速度(mm/s): 整场比赛所有走位的默认“车速”。
   * 取值范围/推荐: 100~600; 当前 200(兼顾速度与到位精度)。
   * 影响: 调大→跑得快但起步冲、到点刹停距离长、过坡/对准易超调;
   *       调小→稳但不赶时间时更稳, 比赛时间紧张时可酌情加大(如 250~300)。
   * 若只想让某一段更快, 不必改这里: 在 MissionControl.c 路线宏旁用
   * Chassis_Move_* 前的 Chassis_SetMaxSpeed 单独提速即可。 */
  Chassis_SetMaxSpeed(200);

  /* ---- ⭐ 注入“摆臂阻塞期间刷新陀螺仪 yaw”的回调 ----
   * Arm_GotoPose() 摆一次臂要阻塞好几秒; 期间底盘闭环跑在 TIM9 中断里,
   * 如果不继续刷 yaw, 航向/转向闭环会读到冻结角度(原地转向会转不完 → 车自转)。
   * 详见 MissionControl.c 的 Mission_Coop_Wait()。 */
  Mission_SetYawPollHook(ArmWait_PollYaw);

  elog_i("MAIN", "KEY1=Start  KEY2=YawZero  KEY2_LONG=ArmTeach");
#endif /* !ARM_UART_TEST_MODE */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LED_ON();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if ARM_UART_TEST_MODE
    /* ===== 机械臂串口自检模式 =====
     * 小车运动代码(底盘演示/任务状态机)整段不参与执行, 小车不会动。 */
    /* KEY1 按下 -> 对当前 ID 的舵机跑一轮串口收发测试(不发位置指令, 舵机不动) */
    if (Key_Check(KEY_1, KEY_DOWN))
    {
      ArmUartTest_Run();
    }
    /* KEY1 长按(约2s) -> 追加一次"动作测试"(需 ARM_TEST_MOVE_ENABLE=1, 否则只提示) */
    if (Key_Check(KEY_1, KEY_LONG))
    {
      ArmUartTest_MoveSelected();
    }
    /* K EY2 按下 -> 切换到下一个待测舵机 ID (1->2->3->4->5->1) */
    if (Key_Check(KEY_2, KEY_DOWN))
    {
      ArmUartTest_NextServo();
    }

    /* 每 200ms 刷新一次测试结果到 OLED */
    if (HAL_GetTick() - oledTick >= 200)
    {
      oledTick = HAL_GetTick();
      ArmUartTest_ShowOled();
    }
#else
    /* KEY1 按下 -> 启动/停止底盘演示 (原 MotorTest 已注释) */
    // if (Key_Check(KEY_1, KEY_DOWN))
    // {
    //   MotorTest();
    //   demoRun = !demoRun;
    //   if (demoRun)
    //   {
    //     Mission_Init();      /* 停止任务, 避免与演示冲突 */
    //     Chassis_Demo_Reset();
    //     elog_i("MAIN", "KEY1: demo START");
    //   }
    //   else
    //   {
    //     Chassis_Stop();
    //     elog_i("MAIN", "KEY1: demo STOP");
    //   }
    // }
    /* ---- 机械臂示教标定模式(联调期): KEY2 长按 进入/退出 ---- */
#if !MISSION_TEST_NO_ARM
    if (Key_Check(KEY_2, KEY_LONG))
    {
      if (ArmTeach_IsActive())
      {
        ArmTeach_Exit();
        elog_i("MAIN", "Arm teach EXIT (back to mission mode)");
      }
      else
      {
        ArmTeach_Enter();
        elog_i("MAIN", "Arm teach ENTER (KEY2=next, KEY1=write)");
      }
    }

    if (ArmTeach_IsActive())
    {
      /* 示教中: KEY2 = 切换动作数组, KEY1 = 读取当前位置并写入当前动作 */
      if (Key_Check(KEY_2, KEY_DOWN))
      {
        ArmTeach_NextAction();
      }
      if (Key_Check(KEY_1, KEY_DOWN))
      {
        ArmTeach_WriteCurrent();
      }
    }
    else
#endif /* !MISSION_TEST_NO_ARM */
    {
#if MISSION_DEBUG_ARM_SEQ
      /* ---- 机械臂动作单独调试: 底盘完全不动 ----
       * KEY1 第一次按 = 启动调试序列(step0), 之后每按一次 = 切到下一个动作。
       * ⚠️ 以前这里把 Mission_DebugArmStart() 整段注释掉了, 而
       *    Mission_DebugArmUpdate() 开头就是 if(s_armdbg_idle) return;
       *    → 序列永远处于“空闲”, 按 KEY1 不会执行任何动作。 */
      if (Key_Check(KEY_1, KEY_DOWN))
      {
        // if (Mission_DebugArmIsIdle())
        // {
        //   // Mission_DebugArmStart();   /* 尚未开始: 进入序列 */
        // }
        // else
        // {
          Mission_ChangeStep();      /* 已在序列中: 切到下一个动作 */
        // }
      }
      Mission_DebugArmUpdate();

#elif MISSION_DEBUG_VISION_TASK
#if MISSION_DEBUG_VISION_TASK == 5
      /* ---- 视觉串口链路监控: KEY1长按=scan_qr, KEY1=run_task:1, KEY2=reset:0 ---- */
      if (Key_Check(KEY_1, KEY_LONG))
      {
        Mission_DebugVisionLinkSend(1);
      }
      if (Key_Check(KEY_1, KEY_DOWN))
      {
        Mission_DebugVisionLinkSend(2);
      }
      if (Key_Check(KEY_2, KEY_DOWN))
      {
        Mission_DebugVisionLinkSend(0);
      }
#else
      /* ---- 视觉单独调试: KEY1 触发一次对应目标对准(不跑路线、不动机械臂) ---- */
      if (Key_Check(KEY_1, KEY_DOWN))
      {
        Mission_DebugVisionStart();
      }
#endif
      Mission_DebugVisionUpdate();
#elif CHASSIS_ENC_CALIB
      /* ================= 编码器静态标定 (人工推车, 小车不会自己动) =================
       * 目标: 测"每轮走同样距离各报多少编码器计数" → 标定 Chassis.h 的
       *       CH_WHEEL_SCALE_*(报告里乘 1000 打印, 如 1027 → 填 1.027f)。
       * 步骤:
       *   ① KEY1 → 清零(内部自动挂起底盘, 电机不输出、不跑任务/机械臂)
       *   ② 用卷尺在地上量出 ENC_CALIB_DIST_MM, 沿【车头方向】把车直线推过去;
       *      推的过程中蓝牙每 500ms 打一次累计计数(live), 推到 1000mm 约 6622
       *   ③ KEY2 → 打印 I/CALIB 两行(四轮计数 + 建议系数)
       *   ④ 反向再推一次对比: 两次系数一致(差<0.5%)才用静态标定;
       *      两次差得多 / "有时 M1 多有时 M2 多" → 不是 counts/mm 问题, 别改
       *   ⑤ KEY1 长按 → 结束标定, 恢复正常闭环(长按时会先触发一次清零, 无妨)
       * ⚠️ 标定要开着 EIDE 重载+构建烧录, 完成后记得把 CHASSIS_ENC_CALIB 改回 0 */
      Chassis_CalibPoll();   /* 必须频繁调: 每次取一次编码器增量并累计 */
      if (Key_Check(KEY_1, KEY_DOWN))
      {
        Chassis_CalibStart();
        elog_i("CALIB", "已清零: 沿车头方向直线推 %.0f mm 后按 KEY2", (double)ENC_CALIB_DIST_MM);
      }
      if (Key_Check(KEY_1, KEY_LONG))
      {
        Chassis_CalibStop();
        elog_i("CALIB", "标定结束, 已恢复正常闭环");
      }
      if (Key_Check(KEY_2, KEY_DOWN))
      {
        Chassis_CalibReport((float)ENC_CALIB_DIST_MM);
      }
      if (Key_Check(KEY_2, KEY_LONG))
      {
        /* ---- 闭环直行, 测"实际走了多远"(修全局比例, 用卷尺量) ----
         * 用法: 车头对准卷尺 0 刻度 → 长按 KEY2 → 停稳后读卷尺实际距离 D_act。
         *       多测 2~3 次取平均(每次先按 KEY1 挂起、摆回 0 刻度)。
         * 结论怎么用:
         *   ① 四轮一起偏 (比如永远走 1015mm) → 是全局比例: 把四个
         *      CH_WHEEL_SCALE_* 同时乘 1000/D_act(≈0.985), 或者把
         *      CH_WHEEL_DIAMETER_MM 从 75 改成 75*D_act/1000(≈76.2)。
         *   ② 距离对但不直 → 那是轮间差异/打滑, 看 I/MOVE 日志的 e: 和 yaw。
         * ⚠️ 每次都会直行 1m, 留够场地。 */
        Chassis_CalibStop();                                          /* 退出挂起, 恢复闭环 */
        Chassis_SetHeadingRef(jy61p ? jy61p->var.yaw : 0.0f);         /* 以当前朝向为准, 免得边跑边纠 */
        Chassis_Move_Forward(ENC_CALIB_DIST_MM);
        elog_i("CALIB", "已直行 %d mm: 停稳后量卷尺 D_act, 看四个 scale 是否都要乘 %d/D_act",
               (int)ENC_CALIB_DIST_MM, (int)ENC_CALIB_DIST_MM);
      }
      if (HAL_GetTick() - dbgTick >= 500)
      {
        dbgTick = HAL_GetTick();
        elog_i("CALIB", "live(前进为正): FL=%ld FR=%ld BL=%ld BR=%ld",
               (long)Chassis_CalibAccum(0), (long)Chassis_CalibAccum(1),
               (long)Chassis_CalibAccum(2), (long)Chassis_CalibAccum(3));
      }
#else
      /* KEY1 按下 -> 启动比赛任务 */
      if (Key_Check(KEY_1, KEY_DOWN))
      {
        demoRun = false;
        if (HAL_GetTick() < JY_WARMUP_MS)
        {
          /* 陀螺仪未稳定前不启动任务, 保证每次上电跑法一致 */
          elog_i("MAIN", "JY61P warming up (%lu/%d ms), wait...",
                 (unsigned long)HAL_GetTick(), (int)JY_WARMUP_MS);
        }
        else
        {
          /* 启动前把当前朝向归零: 每次上电都以放置朝向为 0°, 跑法一致 */
          if (jy61p != NULL)
          {
            jy61p->fun->YAW_ZERO(jy61p);
            jy61p->var.yaw = 0;
            elog_i("MAIN", "Yaw auto-zero before mission");
          }
          Mission_Init();
          Mission_Start();
          elog_i("MAIN", "KEY1: mission START");
        }
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

      /* 底盘演示(循环: 前进/横移/旋转/后退), KEY1 启动 */
      if (demoRun)
      {
        Chassis_Demo();

        /* 每 200ms 打印底盘调试信息(位置/目标/转向剩余), 方便诊断 */
        if (HAL_GetTick() - dbgTick >= 200)
        {
          dbgTick = HAL_GetTick();
          Chassis_DebugLog();
        }
      }
      else
      {
        Mission_Update();   /* 比赛任务主状态机(非演示时运行) */
      }
#endif /* 调试模式(ARM_SEQ / VISION_TASK) */
    }

    /* 每 20ms 快速读取航向角 yaw, 供底盘转向闭环使用 */
    if (HAL_GetTick() - imuTick >= 20)
    {
      imuTick = HAL_GetTick();
      if (HAL_GetTick() >= JY_WARMUP_MS)
      {
        JY61P_ReadYaw();
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
#if !MISSION_TEST_NO_ARM
      if (ArmTeach_IsActive())
      {
        OLED_ShowString(0, 48, "Teach:", OLED_6X8);
        OLED_ShowString(36, 48, (char *)ArmTeach_GetActionName(), OLED_6X8);
      }
      else
#endif
      {
        OLED_ShowString(0, 48, "S:", OLED_6X8);
        OLED_ShowNum(12, 48, (uint32_t)g_mission_state, 2, OLED_6X8);
      }
      /* 转向环 PID 调试(仅原地转向时有值): Si=I项瞬时, Ip=本次转向 I 项峰值(保持) */
      OLED_ShowString(0,  56, "Si:", OLED_6X8);
      OLED_ShowFloatNum(18, 56, Chassis_GetSteerITerm(), 3, 1, OLED_6X8);
      OLED_ShowString(60, 56, "Ip:", OLED_6X8);
      OLED_ShowFloatNum(78, 56, Chassis_GetSteerIPeak(), 3, 1, OLED_6X8);
      OLED_Update();
    }

    /* 每 500ms 经蓝牙串口(USART3)打印姿态角与编码器计数
     * (调试模式下屏蔽, 免得淹没视觉收发日志) */
#if !MISSION_DEBUG_VISION_TASK && !MISSION_DEBUG_ARM_SEQ && !CHASSIS_ENC_CALIB
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
      Chassis_HeadingDebugLog();
      Chassis_SteerDebugLog();   /* 转向环 PID: err/P/I/D/out/积分值 */
    }
#endif
#endif /* !ARM_UART_TEST_MODE */
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
