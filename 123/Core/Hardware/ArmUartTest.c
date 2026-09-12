/**
 * @file    ArmUartTest.c
 * @brief   机械臂串口(USART2)收发自检 —— 实现
 *
 * 每按一次 KEY1, 对当前 ID 的舵机依次做 4 件事, 每件都在 OLED + 日志里给 OK/NG:
 *
 *   [1] 裸字节收发: 手工拼一条 Ping 帧(FF FF ID 02 01 SUM), 直接丢给 Uart_Send,
 *       再把总线上回来的字节原样收下并按十六进制打印。
 *       —— 这一项不经过 SCS 协议库, 最能暴露"TX/RX 接反/没接/没共地/
 *          波特率不对/舵机没供电"这类硬件问题。
 *
 *   [2] SCS 库 Ping: 库内部会校验帧头/ID/帧长/校验和 → 验证"接收 + 解析"链路。
 *
 *   [3] 写指令 + 应答: 写舵机的加速度寄存器(不改变位置, 不会产生动作),
 *       舵机回一帧应答才算通过 → 验证"发送"方向确实被舵机收到。
 *
 *   [4] 读反馈: 读位置/电压/温度 → 验证读指令 + 数据解析(顺带看舵机供电是否正常)。
 *
 * 说明: MCU 的 TX/RX 分别接驱动板, 正常不会收到自己发出去的数据(没有回显),
 *       所以 [1] 收到的字节一定是舵机的应答; 若想单独验证 MCU 串口本身,
 *       可把驱动板侧或接线端子的 TX/RX 短接做回环测试(硬件操作, 代码无需改)。
 */

#include "ArmUartTest.h"
#include "main.h"
#include "uart.h"
#include "OLED.h"
#include "elog.h"
#include "SCServo.h"   /* SCS.h / SMS_STS.h: Ping / writeByte / ReadPos ... */
#include <stdio.h>
#include <string.h>

/* 最近一轮测试结果 */
static ArmUartTest_t s_t;

/* ==================== 内部工具 ==================== */

/**
 * @brief  清空上一轮结果(保留当前 ID 与已跑轮数)
 */
static void ArmTest_ClearResult(void)
{
    s_t.txLen = 0;
    s_t.rxLen = 0;
    memset(s_t.rxRaw, 0, sizeof(s_t.rxRaw));
    s_t.passRaw = 0;

    s_t.pingRet = -1;
    s_t.passPing = 0;

    s_t.writeRet = -1;
    s_t.passWrite = 0;

    s_t.pos = -1;
    s_t.volt = -1;
    s_t.temper = -1;
    s_t.passRead = 0;

    s_t.passAll = 0;
}

/**
 * @brief  手工组装一条 Ping 指令帧(不依赖 SCS 库)
 * @note   帧格式: FF FF ID LEN INST SUM, Ping 的 LEN = 2, INST = 0x01
 *         校验和 SUM = ~(ID + LEN + INST)
 * @retval 帧长度(固定 6)
 */
static uint16_t ArmTest_BuildPingFrame(uint8_t id, uint8_t *out)
{
    out[0] = 0xFF;
    out[1] = 0xFF;
    out[2] = id;
    out[3] = 0x02;   /* LEN = 参数个数(0) + 2 */
    out[4] = 0x01;   /* INST_PING */
    out[5] = (uint8_t)(~(uint8_t)(id + 0x02 + 0x01));
    return 6;
}

/**
 * @brief  字节数组 -> 十六进制字符串(空格分隔), 便于日志打印
 */
static void ArmTest_HexToStr(const uint8_t *buf, uint16_t len, char *out, uint16_t outSize)
{
    static const char HEX[] = "0123456789ABCDEF";
    uint16_t i;
    uint16_t p = 0;

    if (outSize == 0) {
        return;
    }

    for (i = 0; i < len; i++) {
        if ((p + 4) >= outSize) {   /* 预留 1 字节给结束符 */
            break;
        }
        out[p++] = HEX[(buf[i] >> 4) & 0x0F];
        out[p++] = HEX[buf[i] & 0x0F];
        out[p++] = ' ';
    }
    if (p > 0) {
        p--;                        /* 去掉末尾多余的空格 */
    }
    out[p] = '\0';
}

/**
 * @brief  裸收发: 发出 tx, 再把应答按字节收进 rx
 * @param  timeoutMs 每收 1 字节的等待上限; 收不到就认为"应答结束/没有应答"
 * @retval 实际收到的字节数
 */
static uint16_t ArmTest_RawExchange(const uint8_t *tx, uint16_t txLen,
                                    uint8_t *rx, uint16_t rxMax, uint32_t timeoutMs)
{
    uint16_t n = 0;

    Uart_Flush();   /* 丢掉残留数据, 保证收到的都是本次的应答 */

    /* Uart_Send 内部: 阻塞发送 -> 等 TC 发完(纯串口, 无方向切换) */
    Uart_Send((uint8_t *)tx, txLen);

    while (n < rxMax) {
        if (Uart_Read(&rx[n], 1, timeoutMs) != 1) {
            break;  /* 超时且无新数据 */
        }
        n++;
    }
    return n;
}

/* ==================== 对外接口 ==================== */

void ArmUartTest_Init(void)
{
    memset(&s_t, 0, sizeof(s_t));
    s_t.servoId = ARM_TEST_ID_MIN;
    s_t.pingRet = -1;
    s_t.writeRet = -1;
    s_t.pos = -1;
    s_t.volt = -1;
    s_t.temper = -1;

    /* 舵机串口: 波特率/GPIO 已由 MX_USART2_UART_Init 配好(115200),
     * 这里只做一件事: 打开 RXNE 接收中断(板载 485 已在 Uart_Init 里关掉) */
    Uart_Init(115200);
    Uart_Flush();

    elog_i("ARM", "===== Arm UART test mode (car motion DISABLED) =====");
    elog_i("ARM", "Servo bus: USART2 PD5=TX / PD6=RX 115200 -> Feetech driver board");
    elog_i("ARM", "KEY1=press:run test  KEY1=hold(2s):move test[en=%d]  KEY2:next ID",
           (int)ARM_TEST_MOVE_ENABLE);
    elog_i("ARM", "Current servo ID=%u, press KEY1 to start", (unsigned)s_t.servoId);

    ArmUartTest_ShowOled();
}

void ArmUartTest_NextServo(void)
{
    if (s_t.servoId >= ARM_TEST_ID_MAX) {
        s_t.servoId = ARM_TEST_ID_MIN;
    } else {
        s_t.servoId++;
    }
    ArmTest_ClearResult();

    elog_i("ARM", "Select servo ID=%u (press KEY1 to test)", (unsigned)s_t.servoId);
    ArmUartTest_ShowOled();
}

void ArmUartTest_Run(void)
{
    uint8_t  tx[8];
    uint8_t  id = s_t.servoId;
    uint16_t txLen;
    char     hex[3 * 16 + 8];

    ArmTest_ClearResult();
    s_t.runCount++;

    /* ---------- [1] 裸字节发送 + 原始接收 ---------- */
    txLen = ArmTest_BuildPingFrame(id, tx);
    s_t.txLen = txLen;
    s_t.rxLen = ArmTest_RawExchange(tx, txLen, s_t.rxRaw, (uint16_t)sizeof(s_t.rxRaw),
                                    ARM_TEST_RAW_TIMEOUT_MS);

    /* 应答应为 FF FF ID 02 ERR SUM: 收到 >=6 字节且第 3 字节 = ID, 即认为通 */
    s_t.passRaw = (uint8_t)((s_t.rxLen >= 6) &&
                            (s_t.rxRaw[0] == 0xFF) &&
                            (s_t.rxRaw[1] == 0xFF) &&
                            (s_t.rxRaw[2] == id));

    ArmTest_HexToStr(tx, txLen, hex, sizeof(hex));
    elog_i("ARM", "[ID%u][1] RAW TX %uB: %s", (unsigned)id, (unsigned)s_t.txLen, hex);

    ArmTest_HexToStr(s_t.rxRaw, s_t.rxLen, hex, sizeof(hex));
    elog_i("ARM", "[ID%u][1] RAW RX %uB: %s -> %s",
           (unsigned)id, (unsigned)s_t.rxLen, hex, s_t.passRaw ? "OK" : "NG");
    if (!s_t.passRaw) {
        elog_w("ARM", "[1] NG: check PD5->board RX, PD6<-board TX, GND, servo power, 115200");
    }

    /* ---------- [2] SCS 库 Ping(会校验整帧 + 校验和) ---------- */
    s_t.pingRet = Ping(id);
    s_t.passPing = (uint8_t)(s_t.pingRet == (int32_t)id);
    elog_i("ARM", "[ID%u][2] Ping -> %d (%s)",
           (unsigned)id, (int)s_t.pingRet, s_t.passPing ? "OK" : "NG");
    if (!s_t.passPing) {
        elog_w("ARM", "[2] NG: 收到字符但整帧校验失败 -> 看波特率/总线干扰/多舵机抢答");
    }

    /* ---------- [3] 写指令 + 应答(写加速度寄存器, 不产生动作) ---------- */
    s_t.writeRet = writeByte(id, SMS_STS_ACC, 50);
    s_t.passWrite = (uint8_t)(s_t.writeRet == 1);
    elog_i("ARM", "[ID%u][3] Write ACC -> %d (%s)",
           (unsigned)id, (int)s_t.writeRet, s_t.passWrite ? "OK" : "NG");
    if (!s_t.passWrite) {
        elog_w("ARM", "[3] NG: 舵机没回应答 -> 该 ID 上没舵机/舵机未上电/ID 不是 %u", (unsigned)id);
    }

    /* ---------- [4] 读反馈 ---------- */
    s_t.pos = ReadPos(id);
    s_t.volt = ReadVoltage(id);
    s_t.temper = ReadTemper(id);
    s_t.passRead = (uint8_t)(s_t.pos >= 0);
    elog_i("ARM", "[ID%u][4] pos=%d  volt=%d(0.1V)  temp=%dC (%s)",
           (unsigned)id, (int)s_t.pos, (int)s_t.volt, (int)s_t.temper,
           s_t.passRead ? "OK" : "NG");

    /* ---------- 汇总 ---------- */
    s_t.passAll = (uint8_t)(s_t.passRaw && s_t.passPing && s_t.passWrite && s_t.passRead);
    elog_i("ARM", "[ID%u] == %s == (raw=%u ping=%u write=%u read=%u)",
           (unsigned)id, s_t.passAll ? "PASS" : "FAIL",
           (unsigned)s_t.passRaw, (unsigned)s_t.passPing,
           (unsigned)s_t.passWrite, (unsigned)s_t.passRead);

    ArmUartTest_ShowOled();
}

void ArmUartTest_MoveSelected(void)
{
#if !ARM_TEST_MOVE_ENABLE
    /* 动作测试开关关闭(默认): 只提示, 完全不碰舵机, 避免意外动作 */
    elog_w("ARM", "[ID%u] move test disabled: set ARM_TEST_MOVE_ENABLE=1 in ArmUartTest.h",
           (unsigned)s_t.servoId);
#else
    uint8_t id = s_t.servoId;
    int     pos;
    int     target;
    int     ret;

    pos = ReadPos(id);
    if (pos < 0) {
        elog_e("ARM", "[ID%u] move test aborted: read position failed", (unsigned)id);
        return;
    }

    target = pos + ARM_TEST_MOVE_STEPS;
    if (target > 4095) {
        target = 4095;
    }

    EnableTorque(id, 1);   /* 先使能扭矩, 否则舵机不理会位置指令 */

    ret = WritePosEx(id, (int16_t)target, ARM_TEST_MOVE_SPEED, 50);
    elog_i("ARM", "[ID%u] move %d -> %d, ack=%d", (unsigned)id, pos, target, ret);
    HAL_Delay(ARM_TEST_MOVE_TIME_MS);

    ret = WritePosEx(id, (int16_t)pos, ARM_TEST_MOVE_SPEED, 50);
    elog_i("ARM", "[ID%u] move back -> %d, ack=%d", (unsigned)id, pos, ret);
    HAL_Delay(ARM_TEST_MOVE_TIME_MS);

    s_t.pos = ReadPos(id);
    elog_i("ARM", "[ID%u] after move: pos=%d (expect ~%d)", (unsigned)id, (int)s_t.pos, pos);

    ArmUartTest_ShowOled();
#endif /* ARM_TEST_MOVE_ENABLE */
}

const ArmUartTest_t *ArmUartTest_Get(void)
{
    return &s_t;
}

void ArmUartTest_ShowOled(void)
{
    char line[24];

    OLED_Clear();
    OLED_ShowString(0, 0, "ArmUartTest", OLED_8X16);

    snprintf(line, sizeof(line), "ID:%u/%u RUN:%lu",
             (unsigned)s_t.servoId, (unsigned)ARM_TEST_ID_MAX,
             (unsigned long)s_t.runCount);
    OLED_ShowString(0, 16, line, OLED_6X8);

    snprintf(line, sizeof(line), "TX:%u RX:%u %s",
             (unsigned)s_t.txLen, (unsigned)s_t.rxLen, s_t.passRaw ? "OK" : "NG");
    OLED_ShowString(0, 24, line, OLED_6X8);

    snprintf(line, sizeof(line), "Ping:%d WR:%d",
             (int)s_t.pingRet, (int)s_t.writeRet);
    OLED_ShowString(0, 32, line, OLED_6X8);

    snprintf(line, sizeof(line), "POS:%d", (int)s_t.pos);
    OLED_ShowString(0, 40, line, OLED_6X8);

    snprintf(line, sizeof(line), "V:%d T:%dC", (int)s_t.volt, (int)s_t.temper);
    OLED_ShowString(0, 48, line, OLED_6X8);

    OLED_ShowString(0, 56, "K1:go K2:ID hl:mv", OLED_6X8);

    OLED_Update();
}
