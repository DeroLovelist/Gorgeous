# OLED 显示模块 (0.96寸 I2C)

基于江协科技开源驱动，移植到 STM32F407 HAL + 硬件 I2C2 + 双缓冲（阻塞传输，无需 DMA）。

## 文件清单

| 文件          | 说明                                     |
| ------------- | ---------------------------------------- |
| `OLED.h`      | 公开 API 声明、`OLED_DisplayBuf` 指针    |
| `OLED.c`      | 驱动实现：HAL I2C 命令 + 双缓冲阻塞页传输 |
| `OLED_Data.h` | 字模结构定义、外部字库声明               |
| `OLED_Data.c` | ASCII 字库 F8x16/F6x8、图像数据          |

## 架构

```
  main loop
  ┌─────────┐
  │ ShowXxx │──写入──▶ OLED_DisplayBuf (写缓冲)
  │ ShowStr │
  │ ...     │
  └────┬────┘
       │
  OLED_Update()            ← 交换 写缓冲 ↔ 发送缓冲
   └─ 逐页阻塞发送 8 页 ───▶ I2C2 → OLED 屏幕
```

### 双缓冲

- **写缓冲** (`OLED_DisplayBuf`): 所有 `OLED_ShowXxx` 等函数绘制到这里。
- **发送缓冲** (`OLED_SendBuf`): `OLED_Update()` 发送到 OLED 的缓冲。
- `OLED_Update()` 调用时交换两个指针，保证发送期间 CPU 可继续写入。

### 发送方式

每页 128 字节，共 8 页。`OLED_Update()` 逐页通过 `HAL_I2C_Master_Transmit`
阻塞发送（全屏刷新约 24ms @400kHz）。不使用 DMA，也不依赖中断。

## 移植方法（本工程已完成）

1. 将 `OLED/` 文件夹放到 `Core/Hardware/`
2. 在 EIDE 中：
   - 添加包含路径：`Core/Hardware/OLED`
   - 添加源文件：`OLED.c`、`OLED_Data.c`
3. CubeMX 配置 I2C2（本工程 PB10=SCL, PB11=SDA, 400kHz）
   - OLED 使用阻塞 I2C 传输，CubeMX 里配置的 I2C2 DMA/TX 不会被用到（保留无害）

### 更换 I2C 外设

编辑 `OLED.c` 顶部的 `extern` 声明和 `#define OLED_ADDR`：

```c
extern I2C_HandleTypeDef hi2c2; // 改成你的 I2C 句柄
#define OLED_ADDR 0x78          // 从机地址 (7bit: 0x3C << 1)
```

## 初始化

```c
#include "OLED.h"

// I2C2 初始化后（如 main.c 的 MX_I2C2_Init() 之后）：
OLED_Init();

// 主循环中 100ms 刷新一次：
if (HAL_GetTick() - tick >= 100) {
    tick = HAL_GetTick();
    OLED_Clear();
    OLED_ShowString(0, 0, "Hello!", OLED_8X16);
    OLED_Update();  // 交换缓冲 + 阻塞发送
}
```

## API

### 更新（必须调用才能显示）

| 函数            | 说明                         |
| --------------- | ---------------------------- |
| `OLED_Update()` | 全屏刷新（双缓冲，阻塞 I2C） |

### 显存控制

| 函数           | 说明     |
| -------------- | -------- |
| `OLED_Clear()` | 清空显存 |

### 显示

| 函数                                       | 说明             |
| ------------------------------------------ | ---------------- |
| `OLED_ShowChar(x,y,ch,size)`               | 显示字符         |
| `OLED_ShowString(x,y,str,size)`            | 显示字符串       |
| `OLED_ShowNum(x,y,num,len,size)`           | 显示无符号十进制 |
| `OLED_ShowSignedNum(x,y,num,len,size)`     | 显示有符号十进制 |
| `OLED_ShowHexNum(x,y,num,len,size)`        | 显示十六进制     |
| `OLED_ShowBinNum(x,y,num,len,size)`        | 显示二进制       |
| `OLED_ShowFloatNum(x,y,num,int,frac,size)` | 显示浮点数       |
| `OLED_ShowImage(x,y,w,h,img)`              | 显示图像         |

`size` 取值：`OLED_8X16` (8×16) 或 `OLED_6X8` (6×8)
