/*
 * uart.c
 * UART接口 - 飞特舵机串口 USART4(由 CubeMX 初始化)
 * PC10 = UART4_TX, PC11 = UART4_RX, AF8
 * 接收中断(RXNE) + 环形缓冲区
 */

#include "stm32f4xx.h"
#include "usart.h"   /* CubeMX 生成的 huart4 */
#include "uart.h"

/* 接收环形缓冲区 */
volatile uint8_t uartBuf[UART_BUF_SIZE];
volatile int head = 0;
volatile int tail = 0;

/* 清空接收缓冲区 */
void Uart_Flush(void)
{
    head = tail = 0;
}

/* 初始化: UART4 已由 MX_UART4_Init 完成, 这里只使能 RXNE 接收中断 */
void Uart_Init(uint32_t baudRate)
{
    (void)baudRate;
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
}

/* 阻塞发送 */
void Uart_Send(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart4, buf, len, HAL_MAX_DELAY);
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

/* UART4 接收中断处理: 在 UART4_IRQHandler 中调用 */
void Uart_RxISR(void)
{
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) != RESET) {
        uint8_t ch = (uint8_t)(huart4.Instance->DR & 0xFF);
        uartBuf[tail] = ch;
        tail = (tail + 1) % UART_BUF_SIZE;
        if (tail == head) {         /* 缓冲区满, 丢弃最旧字节 */
            head = (head + 1) % UART_BUF_SIZE;
        }
    }
}
