# EasyLogger 日志系统

> 轻量级嵌入式日志库 V2.2.99（本工程已移植到 STM32F407，输出口 = USART3）

## 一、当前工程移植配置

| 项 | 值 |
|----|----|
| 输出外设 | **USART3**（PD8=TX / PD9=RX，115200-8-N-1） |
| 句柄 | `huart3`（CubeMX 生成的 `UART_HandleTypeDef`） |
| 输出口配置 | `elog_cfg.h` 中 `ELOG_UART_HANDLE = huart3` |
| 波特率 | 115200（与蓝牙模块/串口终端保持一致） |

> 本工程里 USART3 就是"蓝牙串口"（`main.h` 中标签为 `BLU_TX`/`BLU_RX`）。
> 蓝牙模块接到 PD8/PD9（交叉连接）并配成 115200，电脑端配对后即可收到日志（见"五、蓝牙透传"）。

## 二、初始化（必须在 main 中先调用）

```c
#include "elog.h"

elog_init();       // 初始化（会校验 huart3 已就绪）
ELOG_FMT_TABLE();  // 按 elog_cfg.h 里的 ELOG_FMT_TABLE() 设置每级输出格式
elog_start();      // 启动日志
```

> ⚠️ 必须在 `MX_USART3_UART_Init()` **之后**调用，否则 `elog_port_init()` 的断言会失败。

## 三、如何输出日志（两种写法等价）

### 写法 A：直接用 `elog_xxx(tag, fmt, ...)`（推荐，tag 自己传）

```c
elog_i("MAIN", "系统启动, 时钟=%lu MHz", (unsigned long)(SystemCoreClock / 1000000U));
elog_w("ADC",  "电压异常: %.2f V", 3.3);
elog_e("MOTOR","电机 %d 超时", 2);
```

### 写法 B：用 `log_xxx(...)`（tag 来自文件顶部定义的 LOG_TAG）

在使用该宏的 .c 文件顶部、`#include "elog.h"` 之前定义：

```c
#define LOG_TAG "MAIN"
#define LOG_LVL ELOG_LVL_DEBUG   /* 该文件允许输出的最低级别 */
#include "elog.h"

log_i("系统启动, 时钟=%lu MHz", (unsigned long)(SystemCoreClock / 1000000U));
```

> 注意：`log_i("系统启动...")` 的第一个字符串是**格式串**，不是 tag；tag 由 `LOG_TAG` 提供。
> 不要在 `log_i` 里把 `"[MAIN]"` 当第一个参数（那会被当作格式串而打错内容）。

### 级别宏一览

| 宏（elog_ / log_） | 级别 | 颜色(终端) | 说明 |
|---|---|---|---|
| `elog_a` / `log_a` | ASSERT | 品红 | 断言/致命 |
| `elog_e` / `log_e` | ERROR | 红 | 错误 |
| `elog_w` / `log_w` | WARN | 黄 | 警告 |
| `elog_i` / `log_i` | INFO | 青 | 信息 |
| `elog_d` / `log_d` | DEBUG | 绿 | 调试 |
| `elog_v` / `log_v` | VERBOSE | 蓝 | 详细 |

> 输出级别受 `elog_cfg.h` 的 `ELOG_OUTPUT_LVL` 限制（当前 = `ELOG_LVL_INFO`，
> 即 DEBUG/VERBOSE 会被编译掉；要输出更多级别请改这里）。
> 个别文件的 `LOG_LVL` 可额外再压低该文件的输出。

## 四、常用控制 API

```c
void elog_set_filter_lvl(uint8_t level);              // 运行时全局最低级别过滤
void elog_set_filter_tag(const char *tag);            // 只输出某个 tag
void elog_set_filter_kw(const char *keyword);         // 只输出包含关键字的日志
void elog_set_filter(uint8_t lvl, const char *tag, const char *kw);
void elog_set_filter_tag_lvl(const char *tag, uint8_t lvl);

void elog_set_fmt(uint8_t level, size_t set);         // 设置某级输出内容(见 ELOG_FMT_*)
void elog_set_output_enabled(bool enabled);           // 开/关输出
void elog_set_text_color_enabled(bool enabled);       // 开/关 ANSI 颜色
void elog_raw_output(const char *format, ...);        // 原始输出(不带格式头)

void elog_assert_set_hook(void (*hook)(const char*, const char*, size_t)); // 断言钩子
```

### 输出格式位（`elog_set_fmt` 用，对应 `ELOG_FMT_*`）

```
ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME | ELOG_FMT_P_INFO |
ELOG_FMT_T_INFO | ELOG_FMT_DIR | ELOG_FMT_FUNC | ELOG_FMT_LINE
```

格式表 `ELOG_FMT_TABLE()`（见 `elog_cfg.h`）已按 `ASSERT_FMT/ERROR_FMT/...` 配好每级输出内容，一般不用改。

## 五、蓝牙透传到电脑（USART3 → 蓝牙 → PC）

**当前配置完全支持。** 接线与设置如下：

```
MCU PD8 (USART3_TX) ──► 蓝牙模块 RXD
MCU PD9 (USART3_RX) ◄── 蓝牙模块 TXD
                        蓝牙模块 VCC ── 5V/3.3V(按模块要求)
                        蓝牙模块 GND ── GND
```

1. 蓝牙模块默认波特率多为 9600，需用 AT 指令改成 **115200**（与本工程一致）：
   - HC-05：进入 AT 模式后发 `AT+UART=115200,0,0`（具体指令看模块手册）。
   - HC-06：直接发 `AT+BAUD8`（对应 115200）。
2. 电脑端打开蓝牙配对，成功后会出现一个**蓝牙串口(COM)**。
3. 用串口终端（MobaXterm / PuTTY / 串口助手）打开该 COM，波特率选 115200。
4. MCU 上电执行 `elog_init/elog_start` 后，所有 `elog_xxx()` 输出就会经蓝牙发到电脑。

> 波特率对不上会乱码，优先把模块配成 115200。
> `elog_port_output()` 是阻塞式 `HAL_UART_Transmit`，日志量大时占用 CPU，量小无影响。

## 六、常见问题

- **串口没输出**：确认 `MX_USART3_UART_Init()` 先于 `elog_init()`；确认模块波特率 = 115200。
- **想输出 DEBUG/VERBOSE**：把 `elog_cfg.h` 的 `ELOG_OUTPUT_LVL` 改大（如 `ELOG_LVL_VERBOSE`）。
- **颜色/ANSI 乱码**：终端不支持颜色时把 `elog_cfg.h` 的 `ELOG_COLOR_ENABLE` 注释掉。
- **单行被截断**：`ELOG_LINE_BUF_SIZE`（默认 256）决定单行缓冲，超长会截断，可按需调大。
- **想异步/缓冲输出**：打开 `elog_cfg.h` 的 `ELOG_ASYNC_OUTPUT_ENABLE` / `ELOG_BUF_OUTPUT_ENABLE`
  （`elog_async.c` / `elog_buf.c` 已加入编译）。
