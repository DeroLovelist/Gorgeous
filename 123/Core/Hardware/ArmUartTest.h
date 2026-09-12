/**
 * @file    ArmUartTest.h
 * @brief   机械臂串口(USART2)收发自检模块
 *
 * 用途: 调机械臂阶段, 单独验证 "MCU <-> 舵机" 这条串口链路是否正常,
 *       测试期间小车运动代码(底盘闭环 + 任务状态机)全部不启用, 小车不会动。
 *
 * 链路: USART2(PD5=TX / PD6=RX, 115200) -> 飞特官方驱动板(纯串口, 无方向脚) -> 舵机总线
 *
 * 交互(KEY1=PD3, KEY2=PC12):
 *   KEY1 单击 -> 对当前 ID 的舵机跑一轮收发测试(不发位置指令, 舵机不动作)
 *   KEY1 长按 -> 先跑一轮收发测试, 再做"动作测试"(小幅往返一次)
 *               ⚠ 动作测试需 ARM_TEST_MOVE_ENABLE=1, 否则只打印提示
 *   KEY2 单击 -> 切换到下一个舵机 ID (1->2->3->4->5->1)
 *
 * 结果同时输出到: OLED 屏 + elog(USART3 PD8/PD9, 可接蓝牙/串口助手到电脑看)
 */

#ifndef __ARMUARTTEST_H
#define __ARMUARTTEST_H

#include <stdint.h>

/* =====================================================================
 * ⭐⭐ 测试模式总开关 (main.c 用它决定是否屏蔽小车运动代码)
 * ARM_UART_TEST_MODE:
 *   1 = 机械臂串口自检模式: main 里【不初始化底盘、不启动任务状态机、
 *       不调用 ServoArm_Init】, 小车运动相关代码整段不参与执行
 *       → 小车绝对不会动, 只跑本模块的串口收发测试。
 *   0 = 正常比赛模式: 恢复底盘闭环 + Mission 状态机(原逻辑)。
 * 取值范围: 0 / 1; 默认 1(当前正在调机械臂串口)。
 * 影响: 调完机械臂必须改回 0 才是比赛逻辑, 忘记改会出现"上电不跑任务"。
 * ===================================================================== */
#define ARM_UART_TEST_MODE      1

/* ⭐ 动作测试开关:
 *   1 = 允许"KEY1 长按"让当前 ID 的舵机小幅往返一次(舵机会真的动!)
 *   0 = 只做串口收发测试, 舵机全程不动(默认: 先确认通信正常, 再开动作测试)
 * 取值范围: 0 / 1; 影响: 置 1 后测试时机械臂会动, 注意周围别夹到手。 */
#define ARM_TEST_MOVE_ENABLE    0

/* ---- 动作测试参数(仅在 ARM_TEST_MOVE_ENABLE=1 时生效) ---- */
/* 相对当前位置的偏移量(0~4095 单位, 4096≈360°; 100 约 8.8°)
 * 取值范围: 30~500; 影响: 太小肉眼看不出动, 太大会撞到结构件。 */
#define ARM_TEST_MOVE_STEPS     100
/* 运动速度(步/秒, 即舵机 GOAL_SPEED 单位)
 * 取值范围: 100~1000; 影响: 越大越快越猛, 太小可能带不动关节。 */
#define ARM_TEST_MOVE_SPEED     300
/* 每个方向等待时间(ms), 保证动作走完再读位置
 * 取值范围: 300~2000; 影响: 太小 → 还没到位就读位置, 看起来"没回来"。 */
#define ARM_TEST_MOVE_TIME_MS   800

/* 原始收发测试的等待超时(ms): 每收 1 个字节最多等这么久, 收不到即认为再无数据
 * 取值范围: 5~100; 默认 20(115200 下 6 字节应答只需约 0.6ms, 20 足够)。 */
#define ARM_TEST_RAW_TIMEOUT_MS 20

/* 可测试的舵机 ID 范围(与 ServoArm.h 的 SERVO_ARM_ID_MIN/MAX 一致) */
#define ARM_TEST_ID_MIN         1
#define ARM_TEST_ID_MAX         5

/* ---------------- 测试结果 ---------------- */
typedef struct
{
    uint8_t  servoId;     /* 当前测试的舵机 ID */
    uint32_t runCount;    /* 上电以来跑过的测试轮数 */

    uint16_t txLen;       /* [1] 原始发送字节数 */
    uint16_t rxLen;       /* [1] 原始接收字节数 */
    uint8_t  rxRaw[16];   /* [1] 原始接收到的字节(日志里按十六进制打印) */
    uint8_t  passRaw;     /* [1] 原始收发是否通过 */

    int32_t  pingRet;     /* [2] SCS 库 Ping 返回值(期望 = servoId; -1 = 无应答) */
    uint8_t  passPing;    /* [2] 是否通过 */

    int32_t  writeRet;    /* [3] 写指令应答结果(写加速度寄存器; 1 = 舵机已应答) */
    uint8_t  passWrite;   /* [3] 是否通过 */

    int32_t  pos;         /* [4] 读到的位置(-1 = 失败) */
    int32_t  volt;        /* [4] 电压(单位 0.1V, 例如 86 = 8.6V) */
    int32_t  temper;      /* [4] 温度(℃) */
    uint8_t  passRead;    /* [4] 是否通过(以"读位置成功"为准) */

    uint8_t  passAll;     /* 以上 4 项全部通过 */
} ArmUartTest_t;

/**
 * @brief  初始化自检模块: 打开舵机串口(USART2)接收中断
 * @note   需在 MX_USART2_UART_Init() 之后调用; 本函数不会让舵机动作
 */
void ArmUartTest_Init(void);

/**
 * @brief  切换到下一个舵机 ID (ARM_TEST_ID_MIN ~ ARM_TEST_ID_MAX 循环)
 */
void ArmUartTest_NextServo(void);

/**
 * @brief  对当前 ID 的舵机跑一轮"串口收发"测试(4 项), 结果存内部并刷新 OLED
 * @note   不会发送位置指令 → 舵机不会动作
 */
void ArmUartTest_Run(void);

/**
 * @brief  对当前 ID 的舵机做"动作测试"(小幅往返一次)
 * @note   仅当 ARM_TEST_MOVE_ENABLE=1 时真正动作; 否则只打印提示。
 *         会先使能该舵机扭矩, 做完回到动作前的位置
 */
void ArmUartTest_MoveSelected(void);

/**
 * @brief  获取最近一轮测试结果(只读)
 */
const ArmUartTest_t *ArmUartTest_Get(void);

/**
 * @brief  把最近一轮测试结果显示到 OLED
 */
void ArmUartTest_ShowOled(void);

#endif /* __ARMUARTTEST_H */
