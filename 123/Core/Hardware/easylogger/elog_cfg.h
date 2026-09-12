/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015-2016, Armink, <armink.ztl@gmail.com>
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
 * Function: It is the configure head file for this library.
 * Created on: 2015-07-30
 */

#ifndef _ELOG_CFG_H_
#define _ELOG_CFG_H_
/*---------------------------------------------------------------------------*/
/* enable log output. */
#define ELOG_OUTPUT_ENABLE
/* setting static output log level. range: from ELOG_LVL_ASSERT to ELOG_LVL_VERBOSE */
#define ELOG_OUTPUT_LVL ELOG_LVL_INFO
/* enable assert check */
#define ELOG_ASSERT_ENABLE
/* buffer size for every line's log */
#define ELOG_LINE_BUF_SIZE 256
/* output line number max length */
#define ELOG_LINE_NUM_MAX_LEN 5
/* output filter's tag max length */
#define ELOG_FILTER_TAG_MAX_LEN 30
/* output filter's keyword max length */
#define ELOG_FILTER_KW_MAX_LEN 16
/* output filter's tag level max num */
#define ELOG_FILTER_TAG_LVL_MAX_NUM 5
/* output newline sign */
#define ELOG_NEWLINE_SIGN "\n"
/*---------------------------------------------------------------------------*/
/* enable log color */
#define ELOG_COLOR_ENABLE
/* change the some level logs to not default color if you want */
#define ELOG_COLOR_ASSERT  (F_MAGENTA B_NULL S_NORMAL)
#define ELOG_COLOR_ERROR   (F_RED B_NULL S_NORMAL)
#define ELOG_COLOR_WARN    (F_YELLOW B_NULL S_NORMAL)
#define ELOG_COLOR_INFO    (F_CYAN B_NULL S_NORMAL)
#define ELOG_COLOR_DEBUG   (F_GREEN B_NULL S_NORMAL)
#define ELOG_COLOR_VERBOSE (F_BLUE B_NULL S_NORMAL)
/*---------------------------------------------------------------------------*/
/* enable log fmt */
/* comment it if you don't want to output them at all */
#define ELOG_FMT_USING_FUNC
#define ELOG_FMT_USING_DIR
#define ELOG_FMT_USING_LINE
/*---------------------------------------------------------------------------*/
/* enable asynchronous output mode */
// #define ELOG_ASYNC_OUTPUT_ENABLE
/* the highest output level for async mode, other level will sync output */
// #define ELOG_ASYNC_OUTPUT_LVL                    ELOG_LVL_ASSERT
/* buffer size for asynchronous output mode */
// #define ELOG_ASYNC_OUTPUT_BUF_SIZE               (ELOG_LINE_BUF_SIZE * 10)
/* each asynchronous output's log which must end with newline sign */
// #define ELOG_ASYNC_LINE_OUTPUT
/* asynchronous output mode using POSIX pthread implementation */
// #define ELOG_ASYNC_OUTPUT_USING_PTHREAD
/*---------------------------------------------------------------------------*/
/* enable buffered output mode */
// #define ELOG_BUF_OUTPUT_ENABLE
/* buffer size for buffered output mode */
// #define ELOG_BUF_OUTPUT_BUF_SIZE (ELOG_LINE_BUF_SIZE * 10)
/*---------------------------------------------------------------------------*/
/* ===== 移植配置: 日志输出口 = USART3 (PD8/PD9) ===== */
#ifndef ELOG_UART_HANDLE
#define ELOG_UART_HANDLE huart3
#endif
#ifndef ELOG_TX_TIMEOUT
#define ELOG_TX_TIMEOUT 100
#endif

//=============================================================================
//                      日志格式 开关配置表 ( 1=开启  0=关闭 )
//=============================================================================
// @ref Log format Switch configuration table
// 配置输出内容，枚举类型为：@ref ElogFmtIndex
// 级别            行号 函数名 文件名 线程 进程 时间 标签 级别
/* 注：用十六进制(而非 0b 二进制字面量)，兼容 Keil ARMCC V5 */
#define ASSERT_FMT  0xE7
#define ERROR_FMT   0xC7
#define WARN_FMT    0x67
#define INFO_FMT    0x03
#define DEBUG_FMT   0xE7
#define VERBOSE_FMT 0x20

// 在elog_init()和elog_start()之间调用，配置输出内容
#define ELOG_FMT_TABLE()                             \
    do {                                             \
        elog_set_fmt(ELOG_LVL_ASSERT, ASSERT_FMT);   \
        elog_set_fmt(ELOG_LVL_ERROR, ERROR_FMT);     \
        elog_set_fmt(ELOG_LVL_WARN, WARN_FMT);       \
        elog_set_fmt(ELOG_LVL_INFO, INFO_FMT);       \
        elog_set_fmt(ELOG_LVL_DEBUG, DEBUG_FMT);     \
        elog_set_fmt(ELOG_LVL_VERBOSE, VERBOSE_FMT); \
    } while (0)

/* start EasyLogger */

#endif /* _ELOG_CFG_H_ */
