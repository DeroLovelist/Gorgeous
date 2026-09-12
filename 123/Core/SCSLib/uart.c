/*
 * uart.c
 * UART接口 - 飞特舵机串口 USART2(由 CubeMX 初始化)
 * PD5 = USART2_TX, PD6 = USART2_RX, AF7
 *
 * 当前接法(见下面 UART_USE_MAX485 开关):
 *   不用板载 MAX485 —— USART2 的 TX/RX 直接接飞特官方驱动板的对应引脚,
 *   即 "纯串口 + 无方向脚" 的实现, 与飞特官方库一致。
 * 备用接法(已不用): 经板载 MAX485 转差分 A/B 接舵机总线, PA2 控制收发方向。
 *
 * 接收中断(RXNE) + 环形缓冲区
 */

#include "stm32f4xx.h"
#include "main.h"   /* MAX485_DR_Pin / MAX485_DR_GPIO_Port 宏 */
#include "usart.h"  /* CubeMX 生成的 huart2 */
#include "uart.h"

/* =====================================================================
 * ⭐ 舵机串口接线方式(换硬件时只改这一个开关)
 * UART_USE_MAX485:
 *   0 = 不用板载 MAX485: USART2(PD5/PD6) 直连飞特官方驱动板的 RX/TX
 *       —— 纯串口, 无方向控制脚(当前, 与官方库"串口+无方向脚"实现一致)
 *   1 = 用板载 MAX485(PA2=DR 控制收发方向), 接舵机总线 A/B
 * 影响: 选错 → 指令发得出去但收不到应答(Ping 一律 -1 / 读数全部超时)。
 * ===================================================================== */
#define UART_USE_MAX485   0

/* 接收环形缓冲区 */
volatile uint8_t uartBuf[UART_BUF_SIZE];
volatile int head = 0;
volatile int tail = 0;

#if UART_USE_MAX485
/* ---- 板载 MAX485 收发方向控制 ---- */
static void MAX485_SetTx(void)
{
    HAL_GPIO_WritePin(MAX485_DR_GPIO_Port, MAX485_DR_Pin, GPIO_PIN_SET);
}

static void MAX485_SetRx(void)
{
    HAL_GPIO_WritePin(MAX485_DR_GPIO_Port, MAX485_DR_Pin, GPIO_PIN_RESET);
}
#else
/* ---- 不用板载 MAX485 时: 把它"关掉", 别让它干扰 PD6 ----
 * 板载 485 的 RO(接收器输出) 是推挽输出, 就接在 USART2_RX(PD6) 这个网上:
 *   /RE=0(接收使能) 时 RO 会主动去驱动 PD6, 与驱动板送来的数据"打架"
 *   → 轻则收到乱码, 重则完全收不到。
 * 这里把 PA2 固定为高(DE=1, /RE=1) → RO 变高阻, 把 PD6 让出来。
 * ⚠ 前提: 板上 /RE 是由 PA2 控制的(DE 与 /RE 并接的常见接法)。
 *   若 /RE 是直接接 GND 的(接收器永远使能), 本函数无效 → 必须把 485 芯片拆掉,
 *   或至少把它的 RO 脚(MAX485 第 1 脚)从 PD6 上挑开/割断。
 * 注意: 此时 DE=1, 芯片会往自己的 A/B 上驱动数据, 所以 A/B 端子上的旧舵机线要拔掉。 */
static void MAX485_Disable(void)
{
    HAL_GPIO_WritePin(MAX485_DR_GPIO_Port, MAX485_DR_Pin, GPIO_PIN_SET);
}
#endif

/* 清空接收缓冲区 */
void Uart_Flush(void)
{
    head = tail = 0;
}

/* 初始化: USART2 已由 MX_USART2_UART_Init 完成, 这里处理 485 状态并使能 RXNE */
void Uart_Init(uint32_t baudRate)
{
    (void)baudRate;
#if UART_USE_MAX485
    MAX485_SetRx();
#else
    MAX485_Disable();   /* 关掉板载 485 的接收器, 释放 PD6 */
#endif
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/* 阻塞发送(纯串口: 无方向切换, 只等最后一个字节发完) */
void Uart_Send(uint8_t *buf, uint16_t len)
{
    if (len == 0) {
        return;
    }

#if UART_USE_MAX485
    MAX485_SetTx();
#endif
    HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
    /* 等待最后一个字节从移位寄存器完整发出(用 485 时也靠它保证发完再切接收) */
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {
    }
#if UART_USE_MAX485
    MAX485_SetRx();
#endif
}

/* 从接收缓冲区读取 len 字节, timeout 为毫秒级超时 */
int16_t Uart_Read(uint8_t *buf, uint16_t len, uint32_t timeout)
{
    uint16_t size = 0;
    uint32_t t0 = HAL_GetTick();
    int16_t ch;

    while (size < len) {
        if (head != tail) {
            ch = uartBuf[head];
            head = (head + 1) % UART_BUF_SIZE;
            if (buf) {
                buf[size] = (uint8_t)ch;
            }
            size++;
            t0 = HAL_GetTick();     /* 收到数据, 重置超时起点 */
        }
        if ((HAL_GetTick() - t0) > timeout) {
            break;
        }
    }
    return size;
}

/* USART2 接收中断处理: 在 USART2_IRQHandler 中调用 */
void Uart_RxISR(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
        uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
        uartBuf[tail] = ch;
        tail = (tail + 1) % UART_BUF_SIZE;
        if (tail == head) {         /* 缓冲区满, 丢弃最旧字节 */
            head = (head + 1) % UART_BUF_SIZE;
        }
    }
}
