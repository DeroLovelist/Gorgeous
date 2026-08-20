/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */

#include <assert.h>
#include <elog.h>
#include <stdio.h>
#include <stm32f4xx_hal.h> /* UART_HandleTypeDef, HAL_UART_Transmit */

extern UART_HandleTypeDef huart3;   /* USART3 = easylogger 输出口 */

ElogErrCode elog_port_init(void)
{
    ElogErrCode result = ELOG_NO_ERR;
    assert(ELOG_UART_HANDLE.gState != HAL_UART_STATE_RESET);
    return result;
}

void elog_port_deinit(void)
{
}

void elog_port_output(const char *log, size_t size)
{
    HAL_UART_Transmit(&ELOG_UART_HANDLE, (uint8_t *)log, size, ELOG_TX_TIMEOUT);
}

void elog_port_output_lock(void)
{
    __disable_irq();
}
void elog_port_output_unlock(void)
{
    __enable_irq();
}

/**
 * get current time interface
 *
 * @return current time
 */
const char *elog_port_get_time(void)
{

    /* add your code here */
    static char time_str[16]; // 静态数组，保证返回指针有效
    uint32_t tick = HAL_GetTick();

    snprintf(time_str, sizeof(time_str), "%lu ms", (unsigned long)tick);

    return time_str;
}

/**
 * get current process name interface
 *
 * @return current process name
 */
const char *elog_port_get_p_info(void)
{

    /* add your code here */
    return "main";
}

/**
 * get current thread name interface
 *
 * @return current thread name
 */
const char *elog_port_get_t_info(void)
{

    /* add your code here */
    return "main";
}
