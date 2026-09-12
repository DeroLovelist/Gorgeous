/***************************************************************************************
 * 本程序由江协科技创建并免费开源共享
 * 移植到 STM32F4 HAL 平台 (硬件 I2C2 + 双缓冲阻塞传输)
 *
 * 程序名称：    0.96寸OLED显示屏驱动程序（4针脚I2C接口）
 * 移植平台：    STM32F407VET6
 * 移植时间：    2026.08
 *
 * 江协科技官方网站：        jiangxiekeji.com
 * 江协科技官方淘宝店：      jiangxiekeji.taobao.com
 ***************************************************************************************
 */

#include "OLED.h"
#include <math.h>
#include "stm32f4xx_hal.h"
#include <string.h>

/* OLED 硬件参数 */
#define OLED_ADDR 0x78 /* 从机地址 (7bit: 0x3C << 1)           */
#define OLED_CMD 0x00  /* 控制字节: 命令                        */
#define OLED_DATA 0x40 /* 控制字节: 数据                        */
#define OLED_PAGES 8
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

/* HAL I2C 句柄 */
extern I2C_HandleTypeDef hi2c2;

/*全局变量*********************/

/**
 * OLED 双显存数组
 * 所有显示函数都只对 OLED_DisplayBuf 进行读写
 * 调用 OLED_Update 时将 DisplayBuf 与 SendBuf 交换
 * 然后阻塞发送 SendBuf 到 OLED 硬件，实现无撕裂刷新
 */
static uint8_t OLED_BufA[OLED_PAGES][OLED_WIDTH];
static uint8_t OLED_BufB[OLED_PAGES][OLED_WIDTH];

uint8_t (*OLED_DisplayBuf)[OLED_WIDTH] = OLED_BufA;     /* 公开: 应用层写缓冲   */
static uint8_t (*OLED_SendBuf)[OLED_WIDTH] = OLED_BufB; /* 内部: 待发送缓冲     */

/*********************全局变量*/

/*工具函数*********************/

/*工具函数仅供内部使用*/

/**
 * 函    数：OLED 写命令
 * 参    数：cmd 要写入的命令字节
 * 返 回 值：无
 * 说    明：阻塞 I2C 发送 2 字节（控制 0x00 + 命令），约 50μs @400kHz
 */
static void OLED_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2] = {OLED_CMD, cmd};
    HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, buf, 2, HAL_MAX_DELAY);
}

/**
 * 函    数：OLED 设置光标位置
 * 参    数：Page 指定光标所在的页，范围：0~7
 * 参    数：X 指定光标所在的 X 轴坐标，范围：0~127
 * 返 回 值：无
 * 说    明：OLED 默认的 Y 轴只能 8 个 Bit 为一组写入，即 1 页 = 8 个 Y 轴坐标
 */
static void OLED_SetCursor(uint8_t Page, uint8_t X)
{
    OLED_WriteCmd(0xB0 | Page);              /* 设置页位置             */
    OLED_WriteCmd(0x10 | ((X & 0xF0) >> 4)); /* 设置 X 位置高 4 位     */
    OLED_WriteCmd(0x00 | (X & 0x0F));        /* 设置 X 位置低 4 位     */
}

/**
 * 函    数：OLED 幂运算
 * 参    数：X 底数，Y 指数
 * 返 回 值：X 的 Y 次方
 */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
        Result *= X;
    return Result;
}

/**
 * 函    数：OLED 清除区域
 * 参    数：X 指定区域左上角的 X 轴坐标，范围：-127~127
 * 参    数：Y 指定区域左上角的 Y 轴坐标，范围：-63~63
 * 参    数：Width 指定区域的宽度，范围：0~128
 * 参    数：Height 指定区域的高度，范围：0~64
 * 返 回 值：无
 * 说    明：仅清除显存数组对应区域，不直接操作 OLED 硬件
 */
static void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t i, j;
    for (j = Y; j < Y + Height; j++)
    {
        for (i = X; i < X + Width; i++)
        {
            if (i >= 0 && i <= 127 && j >= 0 && j <= 63)
                OLED_DisplayBuf[j / 8][i] &= ~(0x01 << (j % 8));
        }
    }
}

/*********************工具函数*/

/*页发送函数*******************/

/**
 * 函    数：OLED 发送一页数据
 * 参    数：page 页号，范围：0~7
 * 参    数：data 128 字节数据指针
 * 返 回 值：无
 * 说    明：先设置光标到该页起始列，再拼接控制字节 0x40 + 数据
 *           阻塞时间约 3ms @400kHz I2C（3命令字节 + 129数据字节）
 */
static void OLED_SendPage(uint8_t page, const uint8_t *data)
{
    uint8_t buf[129];
    buf[0] = OLED_DATA;
    memcpy(buf + 1, data, OLED_WIDTH);

    OLED_SetCursor(page, 0);
    HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, buf, OLED_WIDTH + 1, HAL_MAX_DELAY);
}

/*********************页发送函数*/

/*初始化与更新*****************/

/**
 * 函    数：OLED 初始化
 * 参    数：无
 * 返 回 值：无
 * 说    明：使用前必须调用此初始化函数；I2C2 已在 CubeMX 中配置
 */
void OLED_Init(void)
{
    HAL_Delay(1000); /* 等待 OLED 上电稳定                     */

    OLED_WriteCmd(0xAE); /* 设置显示开启/关闭，0xAE 关闭，0xAF 开启 */

    OLED_WriteCmd(0xD5); /* 设置显示时钟分频比/振荡器频率            */
    OLED_WriteCmd(0x80); /* 0x00~0xFF                               */

    OLED_WriteCmd(0xA8); /* 设置多路复用率                          */
    OLED_WriteCmd(0x3F); /* 0x0E~0x3F                               */

    OLED_WriteCmd(0xD3); /* 设置显示偏移                            */
    OLED_WriteCmd(0x00); /* 0x00~0x7F                               */

    OLED_WriteCmd(0x40); /* 设置显示开始行，0x40~0x7F               */

    OLED_WriteCmd(0xA1); /* 设置左右方向，0xA1 正常，0xA0 左右反置  */

    OLED_WriteCmd(0xC8); /* 设置上下方向，0xC8 正常，0xC0 上下反置  */

    OLED_WriteCmd(0xDA); /* 设置 COM 引脚硬件配置                   */
    OLED_WriteCmd(0x12);

    OLED_WriteCmd(0x81); /* 设置对比度                              */
    OLED_WriteCmd(0xFF); /* 0x00~0xFF                               */

    OLED_WriteCmd(0xD9); /* 设置预充电周期                          */
    OLED_WriteCmd(0xF7);

    OLED_WriteCmd(0xDB); /* 设置 VCOMH 取消选择级别                 */
    OLED_WriteCmd(0x30);

    OLED_WriteCmd(0xA4); /* 设置整个显示打开/关闭                   */

    OLED_WriteCmd(0xA6); /* 设置正常/反色显示，0xA6 正常，0xA7 反色 */

    OLED_WriteCmd(0x8D); /* 设置充电泵                              */
    OLED_WriteCmd(0x14);

    OLED_WriteCmd(0xAF); /* 开启显示                               */

    OLED_Clear();  /* 清空显存数组                            */
    OLED_Update(); /* 更新显示，清屏，防止初始化后花屏        */
}

/**
 * 函    数：OLED 全屏更新
 * 参    数：无
 * 返 回 值：无
 * 说    明：交换双缓冲区后逐页阻塞发送到 OLED 硬件
 *           阻塞时间约 8 × 132 × 9 / 400000 ≈ 23.8ms @400kHz I2C
 *           适用于 ≤40Hz 刷新场景；应用层在两次 Update 之间写入 DisplayBuf
 */
void OLED_Update(void)
{
    uint8_t page;

    /* 交换写缓冲 <-> 发送缓冲，实现无撕裂刷新 */
    uint8_t (*tmp)[OLED_WIDTH] = OLED_DisplayBuf;
    OLED_DisplayBuf = OLED_SendBuf;
    OLED_SendBuf = tmp;

    /* 逐页阻塞发送 */
    for (page = 0; page < OLED_PAGES; page++)
    {
        OLED_SendPage(page, OLED_SendBuf[page]);
    }
}

/*********************初始化与更新*/

/*显存操作*********************/

/**
 * 函    数：OLED 全屏清除
 * 参    数：无
 * 返 回 值：无
 * 说    明：将显存数组全部清零；不会直接操作 OLED 硬件
 *           需要调用 OLED_Update 后才能看到清屏效果
 */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
        for (i = 0; i < 128; i++)
            OLED_DisplayBuf[j][i] = 0x00;
}

/*********************显存操作*/

/*显示函数*********************/

/**
 * 函    数：OLED 显示一个字符
 * 参    数：X 指定字符左上角的 X 轴坐标，范围：-127~127
 * 参    数：Y 指定字符左上角的 Y 轴坐标，范围：-63~63
 * 参    数：Char 指定要显示的字符，范围：ASCII 可打印字符
 * 参    数：FontSize 指定字体大小，取值 OLED_8X16 或 OLED_6X8
 * 返 回 值：无
 * 说    明：不会直接操作 OLED 硬件，需要调用 OLED_Update
 */
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
    if (FontSize == OLED_8X16)
        OLED_ShowImage(X, Y, 8, 16, OLED_F8x16[Char - ' ']);
    else if (FontSize == OLED_6X8)
        OLED_ShowImage(X, Y, 6, 8, OLED_F6x8[Char - ' ']);
}

/**
 * 函    数：OLED 显示字符串
 * 参    数：X 指定字符串左上角的 X 轴坐标，范围：-127~127
 * 参    数：Y 指定字符串左上角的 Y 轴坐标，范围：-63~63
 * 参    数：String 指定要显示的字符串，范围：ASCII 可打印字符
 * 参    数：FontSize 指定字体大小，取值 OLED_8X16 或 OLED_6X8
 * 返 回 值：无
 * 说    明：不会直接操作 OLED 硬件，需要调用 OLED_Update
 */
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
    uint16_t i = 0;
    while (String[i] != '\0')
    {
        OLED_ShowChar(X + i * FontSize, Y, String[i], FontSize);
        i++;
    }
}

/**
 * 函    数：OLED 显示数字（十进制无符号）
 * 参    数：X 指定数字左上角的 X 轴坐标，范围：-127~127
 * 参    数：Y 指定数字左上角的 Y 轴坐标，范围：-63~63
 * 参    数：Number 指定要显示的数字，范围：0~4294967295
 * 参    数：Length 指定数字的位数，范围：0~10
 * 参    数：FontSize 指定字体大小，取值 OLED_8X16 或 OLED_6X8
 * 返 回 值：无
 * 说    明：不会直接操作 OLED 硬件，需要调用 OLED_Update
 */
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
        OLED_ShowChar(X + i * FontSize, Y,
                      Number / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
}

/**  * 函    数：OLED 显示有符号十进制数字
 * 参    数：X 指定数字左上角的 X 轴坐标，范围：-127~127
 * 参    数：Y 指定数字左上角的 Y 轴坐标，范围：-63~63
 * 参    数：Number 指定要显示的数字，范围：-2147483648~2147483647
 * 参    数：Length 指定数字的位数（不含符号位），范围：0~10
 * 参    数：FontSize 指定字体大小，取值 OLED_8X16 或 OLED_6X8
 * 返 回 值：无
 * 说    明：不会直接操作 OLED 硬件，需要调用 OLED_Update
 */
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(X, Y, '+', FontSize);
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(X, Y, '-', FontSize);
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)
        OLED_ShowChar(X + (i + 1) * FontSize, Y,
                      Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
}

/**
 * 函    数：OLED 显示十六进制数字
 * 参    数：X 指定数字左上角的 X 轴坐标，范围：-127~127
 * 参    数：Y 指定数字左上角的 Y 轴坐标，范围：-63~63
 * 参    数：Number 指定要显示的数字，范围：0~0xFFFFFFFF
 * 参    数：Length 指定数字的位数，范围：0~8
 * 参    数：FontSize 指定字体大小，取值 OLED_8X16 或 OLED_6X8
 * 返 回 值：无
 * 说    明：不会直接操作 OLED 硬件，需要调用 OLED_Update
 */
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
            OLED_ShowChar(X + i * FontSize, Y, SingleNumber + '0', FontSize);
        else
            OLED_ShowChar(X + i * FontSize, Y, SingleNumber - 10 + 'A', FontSize);
    }
}

/**
 * 函    数：OLED 显示二进制数字
 * 参    数：X 指定数字左上角的 X 轴坐标，范围：-127~127
 * 参    数：Y 指定数字左上角的 Y 轴坐标，范围：-63~63
 * 参    数：Number 指定要显示的数字，范围：0~0xFFFFFFFF
 * 参    数：Length 指定数字的位数，范围：0~32
 * 参    数：FontSize 指定字体大小，取值 OLED_8X16 或 OLED_6X8
 * 返 回 值：无
 * 说    明：不会直接操作 OLED 硬件，需要调用 OLED_Update
 */
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
        OLED_ShowChar(X + i * FontSize, Y,
                      Number / OLED_Pow(2, Length - i - 1) % 2 + '0', FontSize);
}

/**
 * 函    数：OLED 显示浮点数字
 * 参    数：X 指定数字左上角的 X 轴坐标，范围：-127~127
 * 参    数：Y 指定数字左上角的 Y 轴坐标，范围：-63~63
 * 参    数：Number 指定要显示的浮点数
 * 参    数：IntLength 指定整数部分的位数（不含符号位），范围：0~10
 * 参    数：FraLength 指定小数部分的位数，范围：0~9
 * 参    数：FontSize 指定字体大小，取值 OLED_8X16 或 OLED_6X8
 * 返 回 值：无
 * 说    明：不会直接操作 OLED 硬件，需要调用 OLED_Update
 */
void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength,
                       uint8_t FraLength, uint8_t FontSize)
{
    uint32_t PowNum, IntNum, FraNum;
    if (Number >= 0)
        OLED_ShowChar(X, Y, '+', FontSize);
    else
    {
        OLED_ShowChar(X, Y, '-', FontSize);
        Number = -Number;
    }
    IntNum = (uint32_t)Number;
    Number -= IntNum;
    PowNum = OLED_Pow(10, FraLength);
    FraNum = (uint32_t)round(Number * PowNum);
    IntNum += FraNum / PowNum;
    OLED_ShowNum(X + FontSize, Y, IntNum, IntLength, FontSize);
    OLED_ShowChar(X + (IntLength + 1) * FontSize, Y, '.', FontSize);
    OLED_ShowNum(X + (IntLength + 2) * FontSize, Y, FraNum, FraLength, FontSize);
}

/** * 函    数：OLED 显示图像
 * 参    数：X 指定图像左上角的 X 轴坐标，范围：-127~127
 * 参    数：Y 指定图像左上角的 Y 轴坐标，范围：-63~63
 * 参    数：Width 指定图像的宽度，范围：0~128
 * 参    数：Height 指定图像的高度，范围：0~64
 * 参    数：Image 指定要显示的图像数据
 * 返 回 值：无
 * 说    明：图像数据存储格式为纵向 8 点，高位在下，先从左到右再从上到下
 *           不会直接操作 OLED 硬件，需要调用 OLED_Update
 */
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
    uint8_t i, j;
    int16_t Page, Shift;
    OLED_ClearArea(X, Y, Width, Height);
    for (j = 0; j < (Height - 1) / 8 + 1; j++)
    {
        for (i = 0; i < Width; i++)
        {
            if (X + i < 0 || X + i > 127)
                continue;
            Page = Y / 8;
            Shift = Y % 8;
            if (Y < 0)
            {
                Page -= 1;
                Shift += 8;
            }
            if (Page + j >= 0 && Page + j <= 7)
                OLED_DisplayBuf[Page + j][X + i] |= Image[j * Width + i] << Shift;
            if (Page + j + 1 >= 0 && Page + j + 1 <= 7)
                OLED_DisplayBuf[Page + j + 1][X + i] |= Image[j * Width + i] >> (8 - Shift);
        }
    }
}

/*********************显示函数*/

/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com****************/
