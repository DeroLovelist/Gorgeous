# JY61P 姿态传感器驱动 —— 迁移说明

> 本文档说明：如何把原来的 `JY901` 驱动改为 `JY61P` 驱动，并适配 **STM32F407VET6（168MHz）**。
> 新驱动文件：`Lib/Inc/JY61P.h`、`Lib/Src/JY61P.c`（与 `MyI2C` 软件 I2C 驱动配合使用）。

---

## 1. 结论一句话

**完全兼容。** JY61P 与 JY901 属于维特（WIT）同一套标准 I2C 协议，7 位地址同为 `0x50`，寄存器映射完全一致，角度/角速度换算公式一致，YAW 归零流程一致。
当前 G431 工程无需改 `MyI2C`，换到 F407VET6 时也**无需改 JY61P 内部逻辑**，只改芯片头文件与编译宏。

---

## 2. 与 JY901 的差异（只改了这几处）

| 项目 | JY901 原版 | JY61P 新版 |
|---|---|---|
| 结构体/函数命名 | `JY901_Driver` / `JY901_*` | `JY61P_Driver` / `JY61P_*`（避免同工程链接冲突） |
| I2C 写入（`Write2`） | 带 `0xFF 0xAA` 前缀 | **去掉前缀**，符合官方 I2C 写时序：`Start → 地址 → 寄存器 → DataL → DataH → Stop` |
| 磁力计 HX/HY/HZ | JY901 有 | JY61P 为 6 轴，**无磁力计**，寄存器保留但无效 |
| 寄存器映射 | 0x34~0x3F | **完全一致** |
| 默认 I2C 地址 | 0x50 | **0x50** |
| 角度换算 | raw/32768*180 | **一致** |
| 角速度换算 | raw/32768*2000 | **一致** |
| YAW 归零 | 0x69 解锁 → 0x01 置零 → 0x00 保存 | **一致** |

> ⚠️ 唯一实机需验证的点：若用新版 `JY61P_Write2`（去掉 `0xFF 0xAA`）后 YAW 归零不生效，
> 说明模块固件要求带前缀，把 `JY61P_Write2()` 里被注释掉的 `0xFF/0xAA` 两行加回即可（极少见）。

---

## 3. 硬件接线（软件 I2C，任意 GPIO）

| JY61P | 当前 G431 工程（示例） | F407VET6（可任意改） |
|---|---|---|
| SCL | PB10 | 任意 GPIO（如 PB6/PB10） |
| SDA | PB11 | 任意 GPIO（如 PB7/PB11） |
| VCC | 3.3V | 3.3V |
| GND | GND | GND |

> JY61P 与 JY901 均为 3.3V 供电，注意不要接 5V。

---

## 4. 代码使用示例

```c
#include "JY61P.h"

JY61P_Driver *jy61p;

// 创建（地址 0x50，SCL=PB10，SDA=PB11）
jy61p = JY61P_Create(0x50, GPIOB, GPIO_PIN_10, GPIOB, GPIO_PIN_11);

// 初始化（会自动初始化 I2C 引脚）
jy61p->fun->Init(jy61p);

// 读取 Yaw 角度
jy61p->fun->YAW_GET(jy61p);
float yaw = jy61p->var.yaw;

// 陀螺仪归零（上电放置平稳后调用）
jy61p->fun->YAW_ZERO(jy61p);
jy61p->var.yaw = 0;

// 偏航环 PID
jy61p->fun->YAW_PID_SET(jy61p, 0, 0, 0, 0);  // 目标角, P, I, D
jy61p->fun->YAW_PID_OUT(jy61p);              // 输出在 jy61p->var.pid.out
```

> 原 `main.c` 中 `JY901_Driver *jy901` 的用法，直接把类型名、创建函数、`->fun->` 里的调用名
> 换成 `JY61P` 前缀即可，其余逻辑（PID、速度补偿、模式切换）不用动。

---

## 5. 工程编译接入

### 5.1 当前 G431 工程（无需改动 MyI2C）

在 `CMakeLists.txt` 的 `target_sources` 中增加一行：

```cmake
Lib/Src/JY61P.c
```

头文件目录 `Lib/Inc` 已存在，无需新增。若在 `main.c` 使用，`#include "JY61P.h"`。

### 5.2 换到 STM32F407VET6（168MHz）时

`JY61P.c/h` 内部已用条件编译自动选择芯片库：

```c
#if defined(STM32F407xx)
    #include "stm32f4xx.h"
    #include "stm32f4xx_hal.h"
#else   // STM32G431xx 等
    #include "stm32g4xx.h"
    #include "stm32g4xx_hal.h"
#endif
```

所以换 F407 只需在编译宏里定义 **`STM32F407xx`**，无需改 JY61P 文件。

**需要同步改的是 `MyI2C.c` / `MyI2C.h`**（软件 I2C 底层）：

| 文件 | G431（原） | F407（改） |
|---|---|---|
| `MyI2C.h` | `#include "stm32g4xx.h"` | `#include "stm32f4xx.h"` |
| `MyI2C.c` | `#include "stm32g4xx.h"` / `stm32g431xx.h` / `stm32g4xx_hal.h` | `#include "stm32f4xx.h"` / `stm32f407xx.h` / `stm32f4xx_hal.h` |

其余（GPIO 开漏输出、HAL_GPIO 读写、延时循环）在 F4 上完全一致，**时序延时 `for(i<10)` 在 168MHz 下无需调整**。

---

## 6. 注意事项

1. **F407 是 Cortex-M4F（带 FPU）**，`float` 姿态计算没问题，无需额外配置。
2. **JY61P 无磁力计**：航向 Yaw 靠陀螺仪积分，动态会有漂移，静态精度约 0.5°；不要读 `HX/HY/HZ`。
3. **软件 I2C 时序**：若中断（OLED、串口 DMA 等）过于频繁抢占导致 I2C 抖动，可加大 `MyI2C.c` 里 `W_SCL/W_SDA` 的延时循环，或考虑换硬件 I2C。
4. 上电后先调用 `YAW_ZERO()` 归零，并等模块稳定（代码里已有等待循环）。
5. `JY61P` 与 `JY901` 两个驱动**可以同时编译在同一工程**（函数名不同名），方便对比测试，但不要同时对同一组 SCL/SDA 创建两个实例。

---

## 7. 文件清单

```
Lib/Inc/JY61P.h    ← 新增：JY61P 驱动头文件
Lib/Src/JY61P.c    ← 新增：JY61P 驱动源文件
Lib/Inc/MyI2C.h    ← 复用（软件 I2C）
Lib/Src/MyI2C.c    ← 复用（软件 I2C）
```
