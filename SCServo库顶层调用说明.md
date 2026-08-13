# SCServo 库顶层调用说明

本文档说明如何在你的 STM32 工程中**顶层调用**飞特(Feetech) SMS/STS 系列串行舵机库，完成舵机控制与状态读取。

---

## 1. 库文件结构与层次

库位于 `SCSLib/` 目录，按层次从下到上分为四层：

| 文件 | 层次 | 作用 |
|------|------|------|
| `uart.c` / `uart.h` | 硬件层 | UART 收发（HAL 驱动），支持 USART1 / USART2 |
| `SCSerail.c` | 接口层 | 发送缓冲、接收超时、总线切换等待 |
| `SCS.c` / `SCS.h` | 协议层 | SCS 协议帧封装/解析、校验、读写指令 |
| `SMS_STS.c` / `SMS_STS.h` | **应用层** | **舵机内存表的高层封装（推荐直接使用）** |
| `INST.h` | 定义 | 指令码定义 |
| `SCServo.h` | 总头文件 | 一键包含 `INST.h` + `SCS.h` + `SMS_STS.h` |

> **你只需要关心两个东西：**
> 1. `#include "SCServo.h"` —— 引入整个库
> 2. `SMS_STS.h` 里的应用层函数（如 `WritePosEx`、`ReadPos`）

---

## 2. 快速上手（三步）

### 第 1 步：包含头文件

在你要用舵机的 `.c` 文件里：

```c
#include "SCServo.h"   // 舵机库总头文件
#include "uart.h"      // 串口初始化函数
```

### 第 2 步：初始化串口

在 `setup()` 里初始化舵机串口（波特率 **115200**）：

```c
void setup(void)
{
    Uart_Init(115200);   // 舵机总线波特率，必须与舵机一致
    HAL_Delay(1000);     // 上电后等舵机总线稳定
}
```

> 串口选择由全局宏决定：定义了 `USE_USART2_` 就用 USART2（PA2/PA3），
> 定义了 `USE_USART1_` 就用 USART1（PA9/PA10）。

### 第 3 步：调用应用层函数

在 `loop()` 里直接调用 `SMS_STS.h` 中的函数：

```c
void loop(void)
{
    WritePosEx(1, 4095, 2250, 50);  // 让 1 号舵机转到位置 4095
    HAL_Delay(2000);
}
```

---

## 3. 常用 API 速查（应用层 `SMS_STS.h`）

### 3.1 运动控制

| 函数 | 说明 |
|------|------|
| `WritePosEx(ID, Position, Speed, ACC)` | 普通写位置：立即运动到 `Position` |
| `RegWritePosEx(ID, Position, Speed, ACC)` | 异步写位置：先缓存，等 `RegWriteAction()` 触发 |
| `RegWriteAction()` | 触发所有异步写缓存同时执行 |
| `SyncWritePosEx(ID[], IDN, Position[], Speed[], ACC[])` | 同步写位置：一帧控制多个舵机 |
| `WheelMode(ID)` | 切换为恒速（轮式）模式 |
| `WriteSpe(ID, Speed, ACC)` | 恒速模式下的速度控制 |

> 参数说明：
> - `Position`：目标位置，0~4095 对应 0°~360°（带方向位，可传负数）
> - `Speed`：速度，单位步/秒（如 2250）
> - `ACC`：加速度档位（1 档 = 100 步/秒²，如 50 = 5000 步/秒²）

### 3.2 状态读取

| 函数 | 说明 |
|------|------|
| `ReadPos(ID)` | 读当前位置（带方向） |
| `ReadSpeed(ID)` | 读当前速度 |
| `ReadLoad(ID)` | 读当前负载/扭力 |
| `ReadVoltage(ID)` | 读供电电压 |
| `ReadTemper(ID)` | 读温度 |
| `ReadMove(ID)` | 读是否运动中 |
| `ReadCurrent(ID)` | 读电流 |

> 这些函数出错时返回 `-1`，并可通过 `getErr()` 查询通信是否出错
> （`getErr()` 返回 0 = 无错，1 = 通信出错）。

### 3.3 其他

| 函数 | 说明 |
|------|------|
| `EnableTorque(ID, Enable)` | 使能/关闭扭矩（1=使能，0=卸载） |
| `unLockEprom(ID)` / `LockEprom(ID)` | EPROM 解锁/加锁（改 ID、波特率等需先解锁） |
| `CalibrationOfs(ID)` | 中位校准 |
| `Ping(ID)` | 查询舵机是否存在 |
| `FeedBack(ID)` | 一次性读回全部反馈并缓存（配合 `ReadPos(-1)` 等使用） |

---

## 4. 完整示例

```c
#include "stm32f4xx.h"
#include "SCServo.h"
#include "uart.h"

void setup(void)
{
    Uart_Init(115200);   // 初始化舵机串口
    HAL_Delay(1000);
}

void loop(void)
{
    // 1) 写位置：1 号舵机转到 4095，速度 2250，加速度 50
    WritePosEx(1, 4095, 2250, 50);
    HAL_Delay(2270);     // 等待运动完成

    // 2) 写位置：转回 0
    WritePosEx(1, 0, 2250, 50);
    HAL_Delay(2270);

    // 3) 读当前位置（直接读，出错返回 -1）
    int pos = ReadPos(1);
    if (pos != -1) {
        // pos 为当前角度位置
    }
}
```

---

## 5. 常用内存表地址（`SMS_STS.h` 中已定义好宏）

| 地址 | 宏 | 说明 |
|------|-----|------|
| 5 | `SMS_STS_ID` | 舵机 ID |
| 6 | `SMS_STS_BAUD_RATE` | 波特率 |
| 40 | `SMS_STS_TORQUE_ENABLE` | 扭矩使能 |
| 42~43 | `SMS_STS_GOAL_POSITION_L/H` | 目标位置 |
| 44~45 | `SMS_STS_GOAL_TIME_L/H` | 运行时间 |
| 46~47 | `SMS_STS_GOAL_SPEED_L/H` | 目标速度 |
| 56~57 | `SMS_STS_PRESENT_POSITION_L/H` | 当前位置（只读） |
| 58~59 | `SMS_STS_PRESENT_SPEED_L/H` | 当前速度（只读） |
| 60~61 | `SMS_STS_PRESENT_LOAD_L/H` | 当前负载（只读） |
| 62 | `SMS_STS_PRESENT_VOLTAGE` | 当前电压（只读） |
| 63 | `SMS_STS_PRESENT_TEMPERATURE` | 当前温度（只读） |

---

## 6. 注意事项

1. **波特率**：舵机总线默认 115200，必须与 `Uart_Init()` 的参数一致。
2. **半双工**：舵机是单总线半双工，发送和接收共用同一对引脚，不能同时收发。
3. **广播 ID**：`0xFE` 表示广播，对所有舵机生效（此时不等待应答）。
4. **返回值约定**：协议层写/读函数返回 `1`/`0` 表示成功/失败；应用层读取函数失败返回 `-1`。
5. **同步读**：`syncRead*` 系列函数内部用 `malloc` 申请缓冲区，使用后记得 `syncReadEnd()` 释放。
6. **改 ID / 波特率**：需先用 `unLockEprom(ID)` 解锁，改完用 `LockEprom(ID)` 加锁。





两种做法，都需要在 CubeMX 里配好以下三项：

芯片选 STM32F405xx（必须一致，否则启动文件、链接脚本、HAL 库都对不上）
时钟：HSE 外接晶振 8MHz，PLL 出 168MHz（和 main.c 注释里的“8M外部晶振”一致）
USART2 → Asynchronous 模式，TX=PA2、RX=PA3，波特率 115200，8N1

做法 B：在当前 Keil 工程目录里补一个 .ioc（新建 CubeMX 工程指向同目录）

同样要把 USART2 引脚和 HSE 8MHz 配好，否则重新生成后原配置会被冲掉
生成会覆盖 main.c、stm32f4xx_it.c，记得把 setup()/loop() 和时钟使能放回 USER CODE BEGIN/END 区



#include "stm32f4xx.h"

extern void setup(void);
extern void loop(void);

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;//8MÍâ²¿¾§Ìå
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void initSys(void)
{
	HAL_Init();
	SystemClock_Config();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
}

int main()
{
	initSys();
	setup();
	while(1){
		loop();
	}
}

