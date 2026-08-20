/**
 * 串口模块 (UART4) — 通用调试串口
 * UART4: PC10=TX, PC11=RX, 115200 (由 MX_UART4_Init 配置)
 * 注意: 本模块只用于发送; 舵机库接收走 SCSLib/uart.c 的 Uart_RxISR
 */
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "Serial.h"

extern UART_HandleTypeDef huart4;	/* Core/Src/usart.c 中由 CubeMX 定义 */

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
	/* UART4 已由 MX_UART4_Init 初始化, 无需重复配置 */
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
