#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>
#include <stdio.h>

/* ---- 发送 ---- */
void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

/* ---- 接收(K230 通信) ---- */
/* 接收环形缓冲区大小 */
#define SERIAL_RX_BUF_SIZE  256
/* 单条完整消息(以 \r 或 \n 结尾)最大长度 */
#define K230_LINE_MAX       64

/* 最近收到的一整行文本(已去换行符, 以 \0 结尾) */
extern volatile char g_k230_rx_line[K230_LINE_MAX];
/* 新数据到达标志: 置 1 表示 g_k230_rx_line 已更新, 使用后需手动清 0 */
extern volatile uint8_t g_k230_new_data_flag;

/* 接收中断处理, 在 UART4_IRQHandler 中调用 */
void Serial_RxISR(void);
/* 处理收到的单个字节(按行组装) */
void Serial_ProcessReceivedByte(uint8_t byte);
/* 清空接收缓冲与数据标志 */
void Serial_FlushRx(void);

/* ---- 与 K230 的通信指令(发送) ---- */
void K230_SendLine(const char *line);
void K230_Send_QRCode_Data(const char *qr_data_string); /* 回传二维码数据(旧 K230 用, 新 K230 仅打印) */
void K230_Run_Specific_Task(uint8_t task_number);       /* 让 K230 执行指定任务(1=球 2=靶 3=桶 4=形状) */
/* ⚠️ 新 K230(main.py)在自己的扫码阶段【不读串口】, 所以这个请求已无效;
 *   扫码改为“STM32 摆好 SCAN 姿态后在 STATE_2 等 K230 回 SCAN_OK”。
 *   保留此接口仅为兼容旧版 K230。 */
void K230_Request_QRScan(void);                          /* 请求 K230 扫码(旧版, 已废弃) */
void K230_Start_Align(void);                             /* 请求进入精对准 */
void K230_Reset(void);                                   /* 复位 K230 */

#endif
