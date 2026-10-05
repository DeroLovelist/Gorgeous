/**
 * 串口模块 (UART4) — 与 K230 视觉模块通信
 * UART4: PC10=TX, PC11=RX, 115200 (由 MX_UART4_Init 配置)
 * 功能: 既能发送也能接收(接收按行解析, 供任务状态机读取)
 */
#include "main.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "Serial.h"

extern UART_HandleTypeDef huart4;	/* Core/Src/usart.c 中由 CubeMX 定义 */

/* K230 按行接收结果 */
volatile char g_k230_rx_line[K230_LINE_MAX] = {0};
volatile uint8_t g_k230_new_data_flag = 0;

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
	g_k230_new_data_flag = 0;
	memset((char *)g_k230_rx_line, 0, sizeof(g_k230_rx_line));

	/* UART4 已由 MX_UART4_Init 初始化, 这里只使能 RXNE 接收中断 */
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	HAL_UART_Transmit(&huart4, &Byte, 1, HAL_MAX_DELAY);
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	HAL_UART_Transmit(&huart4, Array, Length, HAL_MAX_DELAY);
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	Serial_SendArray((uint8_t *)String, (uint16_t)strlen(String));
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i, buf[10];
	for (i = 0; i < Length; i ++)
	{
		buf[i] = (uint8_t)(Number % 10) + '0';
		Number /= 10;
	}
	while (i --)
	{
		Serial_SendByte(buf[i]);
	}
}

/**
  * 函    数：自己封装的 printf 函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
	char String[128];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsnprintf(String, sizeof(String), format, arg);
	va_end(arg);					//结束变量arg
	Serial_SendString(String);		//串口发送字符数组（字符串）
}

/* ================= 接收部分 ================= */

/**
  * 函    数：处理接收到的单个字节(按行组装, 遇 \r / \n 结束一行)
  * 参    数：byte 收到的一个字节
  * 返 回 值：无
  */
void Serial_ProcessReceivedByte(uint8_t byte)
{
	static uint8_t line[K230_LINE_MAX];
	static uint16_t index = 0;

	if (byte == '\n' || byte == '\r')
	{
		if (index > 0)
		{
			line[index] = '\0';
			memcpy((char *)g_k230_rx_line, line, index + 1);
			g_k230_new_data_flag = 1;
		}
		index = 0;
	}
	else
	{
		if (index < (K230_LINE_MAX - 1))
		{
			line[index++] = byte;
		}
		else
		{
			index = 0;   /* 行过长, 防溢出丢弃 */
		}
	}
}

/**
  * 函    数：UART4 接收中断处理
  * 参    数：无
  * 返 回 值：无
  */
void Serial_RxISR(void)
{
	if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) != RESET)
	{
		uint8_t ch = (uint8_t)(huart4.Instance->DR & 0xFF);
		Serial_ProcessReceivedByte(ch);
	}
}

/**
  * 函    数：清空接收缓冲与数据标志
  * 参    数：无
  * 返 回 值：无
  */
void Serial_FlushRx(void)
{
	g_k230_new_data_flag = 0;
	memset((char *)g_k230_rx_line, 0, sizeof(g_k230_rx_line));
}

/* ================= 与 K230 的通信指令(发送) ================= */

/**
  * 函    数：向 K230 发送一行文本(自动加 \n)
  */
void K230_SendLine(const char *line)
{
	if (line == NULL) return;
	Serial_SendString((char *)line);
	Serial_SendString("\n");
}

/**
  * 函    数：回传二维码数据给 K230
  * 备    注：新 K230 的 handle_command 对 "qr_code:" 只做 print, 不再用它
  *           (目标号存在 K230 本地 /sdcard/target.txt), 保留仅为兼容旧版。
  */
void K230_Send_QRCode_Data(const char *qr_data_string)
{
	char buf[64];
	snprintf(buf, sizeof(buf), "qr_code:%s", qr_data_string ? qr_data_string : "");
	K230_SendLine(buf);
}

/**
  * 函    数：指令 K230 执行指定任务(1=排爆 2=打靶 ...)
  */
void K230_Run_Specific_Task(uint8_t task_number)
{
	char buf[32];
	snprintf(buf, sizeof(buf), "run_task:%d", task_number);
	K230_SendLine(buf);
}

/**
  * 函    数：请求 K230 进行一次二维码扫描
  * 备    注：⚠️ 新 K230(main.py)在自己的扫码阶段【不读串口】, 它扫到码后
  *           会主动回一行 "SCAN_OK" 并重启进入 yolo_main。所以这个请求
  *           现在已无效(保留仅为兼容旧版 K230)。STM32 侧改为在 STATE_2 里
  *           等 SCAN_OK, 见 MissionControl.c。
  */
void K230_Request_QRScan(void)
{
	K230_SendLine("scan_qr");
}

/**
  * 函    数：请求 K230 进入精对准模式(周期回传 D:<x>,<y> 误差)
  */
void K230_Start_Align(void)
{
	K230_SendLine("start_align");
}

/**
  * 函    数：复位 K230 到待机状态
  */
void K230_Reset(void)
{
	K230_SendLine("reset:0");
}
